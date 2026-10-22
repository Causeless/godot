/**************************************************************************/
/*  command_queue_mt.h                                                    */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include "core/object/worker_thread_pool.h"
#include "core/os/condition_variable.h"
#include "core/os/mutex.h"
#include "core/os/spin_lock.h"
#include "core/templates/local_vector.h"
#include "core/templates/tuple.h"
#include "core/typedefs.h"

class CommandQueueMT {
	static const size_t MAX_COMMAND_SIZE = 1024;

	struct CommandBase {
		bool sync = false;
		virtual void call() = 0;
		virtual ~CommandBase() = default;

		CommandBase(bool p_sync) :
				sync(p_sync) {}
	};

	template <typename T, typename M, bool NeedsSync, typename... Args>
	struct Command : public CommandBase {
		T *instance;
		M method;
		Tuple<std::decay_t<Args>...> args;

		template <typename... FwdArgs>
		_FORCE_INLINE_ Command(T *p_instance, M p_method, FwdArgs &&...p_args) :
				CommandBase(NeedsSync), instance(p_instance), method(p_method), args(std::forward<FwdArgs>(p_args)...) {}

		void call() override {
			call_impl(BuildIndexSequence<sizeof...(Args)>{});
		}

	private:
		template <size_t... I>
		_FORCE_INLINE_ void call_impl(IndexSequence<I...>) {
			// Move out of the Tuple, this will be destroyed as soon as the call is complete.
			(instance->*method)(std::move(get<I>())...);
		}

		// This method exists so we can call it in the parameter pack expansion in call_impl.
		template <size_t I>
		_FORCE_INLINE_ auto &get() { return ::tuple_get<I>(args); }
	};

	// Separate class from Command so we can save the space of the ret pointer for commands that don't return.
	template <typename T, typename M, typename R, typename... Args>
	struct CommandRet : public CommandBase {
		T *instance;
		M method;
		R *ret;
		Tuple<std::decay_t<Args>...> args;

		_FORCE_INLINE_ CommandRet(T *p_instance, M p_method, R *p_ret, std::decay_t<Args>... p_args) :
				CommandBase(true), instance(p_instance), method(p_method), ret(p_ret), args{ p_args... } {}

		void call() override {
			*ret = call_impl(BuildIndexSequence<sizeof...(Args)>{});
		}

	private:
		template <size_t... I>
		_FORCE_INLINE_ R call_impl(IndexSequence<I...>) {
			// Move out of the Tuple, this will be destroyed as soon as the call is complete.
			return (instance->*method)(std::move(get<I>())...);
		}

		// This method exists so we can call it in the parameter pack expansion in call_impl.
		template <size_t I>
		_FORCE_INLINE_ auto &get() { return ::tuple_get<I>(args); }
	};

	/***** BASE *******/

	static const uint32_t DEFAULT_COMMAND_MEM_SIZE_KB = 64;

	inline static thread_local bool flushing = false;

	// The mutex serializes flushers and guards the sync protocol (the counters below and the condition variable).
	// The command buffer itself is guarded by the spin lock: a push holds it only to write one command, so many
	// threads pushing at once spin for nanoseconds instead of sleeping and waking on the mutex.
	BinaryMutex mutex;
	SpinLock command_lock;
	LocalVector<uint8_t> command_mem;
	ConditionVariable sync_cond_var;
	uint32_t sync_head = 0;
	uint32_t sync_tail = 0;
	uint32_t sync_awaiters = 0;
	WorkerThreadPool::TaskID pump_task_id = WorkerThreadPool::INVALID_TASK_ID;
	uint64_t flush_read_ptr = 0;
	std::atomic<bool> pending{ false };

	template <typename T, typename... Args>
	_FORCE_INLINE_ void create_command(Args &&...p_args) {
		// alloc size is size+T+safeguard
		constexpr uint64_t alloc_size = ((sizeof(T) + 8U - 1U) & ~(8U - 1U));
		static_assert(alloc_size < UINT32_MAX, "Type too large to fit in the command queue.");

		uint64_t size = command_mem.size();
		command_mem.resize(size + alloc_size + sizeof(uint64_t));
		*(uint64_t *)&command_mem[size] = alloc_size;
		void *cmd = &command_mem[size + sizeof(uint64_t)];
		memnew_placement(cmd, T(std::forward<Args>(p_args)...));
		pending.store(true);
	}

	template <typename T, typename... Args>
	_FORCE_INLINE_ void _push_command(Args &&...p_args) {
		command_lock.lock();
		create_command<T>(std::forward<Args>(p_args)...);
		WorkerThreadPool::TaskID pump = pump_task_id;
		command_lock.unlock();

		if (pump != WorkerThreadPool::INVALID_TASK_ID) {
			WorkerThreadPool::get_singleton()->notify_yield_over(pump);
		}
	}

	template <typename T, bool NeedsSync, typename... Args>
	_FORCE_INLINE_ void _push_internal(Args &&...p_args) {
		if constexpr (NeedsSync) {
			// Lock order is always mutex then spin lock, same as the flush.
			MutexLock mlock(mutex);
			_push_command<T>(std::forward<Args>(p_args)...);
			sync_tail++;
			_wait_for_sync(mlock);
		} else {
			_push_command<T>(std::forward<Args>(p_args)...);
		}
	}

	_FORCE_INLINE_ void _prevent_sync_wraparound() {
		bool safe_to_reset = !sync_awaiters;
		bool already_sync_to_latest = sync_head == sync_tail;
		if (safe_to_reset && already_sync_to_latest) {
			sync_head = 0;
			sync_tail = 0;
		}
	}

	void _flush() {
		// Safeguard against trying to re-lock the binary mutex.
		if (flushing) {
			return;
		}

		flushing = true;

		MutexLock lock(mutex);

		if (unlikely(flush_read_ptr)) {
			// Another thread is flushing.
			lock.temp_unlock(); // Not really temp.
			sync();
			flushing = false;
			return;
		}

		alignas(uint64_t) char cmd_local_mem[MAX_COMMAND_SIZE];

		while (true) {
			// Pushes keep appending (and reallocating) while we run, so the buffer is only read under the spin
			// lock and each command is copied out before it is called (see GH-100509).
			command_lock.lock();
			if (flush_read_ptr >= command_mem.size()) {
				command_mem.clear();
				pending.store(false);
				flush_read_ptr = 0;
				command_lock.unlock();
				break;
			}

			uint64_t size = *(uint64_t *)&command_mem[flush_read_ptr];
			flush_read_ptr += sizeof(uint64_t);
			memcpy(cmd_local_mem, (char *)&command_mem[flush_read_ptr], size);
			flush_read_ptr += size;
			command_lock.unlock();

			CommandBase *cmd_local = reinterpret_cast<CommandBase *>(cmd_local_mem);

			lock.temp_unlock();
			cmd_local->call();
			lock.temp_relock();

			if (unlikely(cmd_local->sync)) {
				sync_head++;
				lock.temp_unlock(); // Give an opportunity to awaiters right away.
				sync_cond_var.notify_all();
				lock.temp_relock();
			}

			cmd_local->~CommandBase();
		}

		_prevent_sync_wraparound();

		flushing = false;
	}

	_FORCE_INLINE_ void _wait_for_sync(MutexLock<BinaryMutex> &p_lock) {
		sync_awaiters++;
		uint32_t sync_head_goal = sync_tail;
		do {
			sync_cond_var.wait(p_lock);
		} while (sync_head < sync_head_goal);
		sync_awaiters--;
		_prevent_sync_wraparound();
	}

	void _no_op() {}

public:
	template <typename T, typename M, typename... Args>
	void push(T *p_instance, M p_method, Args &&...p_args) {
		// Standard command, no sync.
		using CommandType = Command<T, M, false, Args...>;
		static_assert(sizeof(CommandType) <= MAX_COMMAND_SIZE);
		_push_internal<CommandType, false>(p_instance, p_method, std::forward<Args>(p_args)...);
	}

	template <typename T, typename M, typename... Args>
	void push_and_sync(T *p_instance, M p_method, Args... p_args) {
		// Standard command, sync.
		using CommandType = Command<T, M, true, Args...>;
		static_assert(sizeof(CommandType) <= MAX_COMMAND_SIZE);
		_push_internal<CommandType, true>(p_instance, p_method, std::forward<Args>(p_args)...);
	}

	template <typename T, typename M, typename R, typename... Args>
	void push_and_ret(T *p_instance, M p_method, R *r_ret, Args... p_args) {
		// Command with return value, sync.
		using CommandType = CommandRet<T, M, R, Args...>;
		static_assert(sizeof(CommandType) <= MAX_COMMAND_SIZE);
		_push_internal<CommandType, true>(p_instance, p_method, r_ret, std::forward<Args>(p_args)...);
	}

	_FORCE_INLINE_ void flush_if_pending() {
		if (unlikely(pending.load())) {
			_flush();
		}
	}

	void flush_all() {
		_flush();
	}

	void sync() {
		push_and_sync(this, &CommandQueueMT::_no_op);
	}

	void wait_and_flush() {
		ERR_FAIL_COND(pump_task_id == WorkerThreadPool::INVALID_TASK_ID);
		WorkerThreadPool::get_singleton()->wait_for_task_completion(pump_task_id);
		_flush();
	}

	void set_pump_task_id(WorkerThreadPool::TaskID p_task_id) {
		command_lock.lock();
		pump_task_id = p_task_id;
		command_lock.unlock();
	}

	CommandQueueMT() {
		command_mem.reserve(DEFAULT_COMMAND_MEM_SIZE_KB * 1024);
	}
};
