/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib.robotcore;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This class implements the TrcEvent. TrcEvent is very important in our event driven asynchronous architecture where
 * things only happen when an event is signaled.
 */
public class TrcEvent
{
    private static final String moduleName = TrcEvent.class.getSimpleName();
    public static final TrcDbgTrace staticTracer = new TrcDbgTrace();

    /**
     * An event has three possible states:
     *  - CLEARED: event should be in this state before starting an asynchronous operation. This is also the default
     *             state when an event is created.
     *  - SIGNALED: when an asynchronous operation is completed, the event is set to this state.
     *  - CANCELED: when an asynchronous operation is canceled, the event is set to this state.
     */
    public enum EventState
    {
        CLEARED,
        SIGNALED,
        CANCELED
    }   //enum EventState

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final AtomicReference<EventState> eventState = new AtomicReference<>(EventState.CLEARED);

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param state specifies the initial state of the event.
     */
    public TrcEvent(String instanceName, EventState state)
    {
        tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        eventState.set(state);
    }   //TrcEvent

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcEvent(String instanceName)
    {
        this(instanceName, EventState.CLEARED);
    }   //TrcEvent

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return "(" + instanceName + "=" + eventState.get() + ")";
    }   //toString

    /**
     * This method clears an event.
     */
    public void clear()
    {
        eventState.set(EventState.CLEARED);
    }   //clear

    /**
     * This method signals an event if it is not canceled.
     */
    public void signal()
    {
        eventState.compareAndSet(EventState.CLEARED, EventState.SIGNALED);
    }   //signal

    /**
     * This method cancels an event if it is not already signaled. An event is either signaled or canceled by the
     * event source either of which will cause whoever is waiting for it to move on. Note: if an event is canceled,
     * no callback will be performed.
     */
    public void cancel()
    {
        eventState.compareAndSet(EventState.CLEARED, EventState.CANCELED);
        // We are canceling an event, remove the callback. We only do callbacks on signaled events, not canceled ones.
        setCallback(null, null);
    }   //cancel

    /**
     * This method checks if the event is signaled.
     *
     * @return true if the event is signaled, false otherwise.
     */
    public boolean isSignaled()
    {
        return eventState.get() == EventState.SIGNALED;
    }   //isSignaled

    /**
     * This method checks if the event was canceled.
     *
     * @return true if the event was canceled, false otherwise.
     */
    public boolean isCanceled()
    {
        return eventState.get() == EventState.CANCELED;
    }   //isCanceled

    //
    // Callback Management.
    //

    /**
     * This interface is implemented by the caller so that it can be notified when the event is signaled.
     */
    public interface Callback
    {
        void notify(Object context);
    }   //interface Callback

    private static class CallbackEventList
    {
        final ArrayList<TrcEvent> eventList = new ArrayList<>();
        final Object listLock = new Object();
    }   //class CallbackEventList

    private static final HashMap<Thread, CallbackEventList> callbackEventListMap = new HashMap<>();
    private Callback callback;
    private Object callbackContext;

    /**
     * This method sets a callback handler so that when the event is signaled, the callback handler is called on
     * the same thread as original caller. This could be very useful if the caller wants to perform some minor actions
     * after an asynchronous operation is completed. Without the callback, the caller would have to set up a state
     * machine waiting for the event to signal, then perform the action. Since the callback is done on the same
     * thread, the caller doesn't have to worry about thread safety. Note: this method is called by another thread
     * on behalf of the original caller.
     *
     * @param thread specifies the thread to do the callback.
     * @param callback specifies the callback handler, null for removing previous callback handler.
     * @param callbackContext specifies the context object passing back to the callback handler.
     */
    public void setCallback(Thread thread, Callback callback, Object callbackContext)
    {
        CallbackEventList callbackEventList;

        if (callback != null)
        {
            // We are setting a callback for the event, let's initialized the event state to clear.
            clear();
        }

        synchronized (callbackEventListMap)
        {
            callbackEventList = callbackEventListMap.get(thread);
        }

        if (callbackEventList != null)
        {
            synchronized (callbackEventList.listLock)
            {
                boolean inList = callbackEventList.eventList.contains(this);

                this.callback = callback;
                this.callbackContext = callbackContext;
                if (callback != null && !inList)
                {
                    // Callback handler is not already in the callback list, add it.
                    callbackEventList.eventList.add(this);
                    tracer.traceDebug(
                        instanceName, "Adding event to the callback list for thread " + thread.getName() + ".");
                }
                else if (callback == null && inList)
                {
                    // Remove the callback from the list.
                    callbackEventList.eventList.remove(this);
                    this.callbackContext = null;
                    tracer.traceDebug(
                        instanceName, "Removing event from the callback list for thread " + thread.getName() + ".");
                }
            }
        }
        else
        {
            tracer.traceWarn(instanceName, "Thread " + thread.getName() + " is not registered.");
            TrcDbgTrace.printThreadStack();
        }
    }   //setCallback

    /**
     * This method sets a callback handler so that when the event is signaled, the callback handler is called on
     * the same thread as this call. This could be very useful if the caller wants to perform some minor actions
     * after an asynchronous operation is completed. Without the callback, the caller would have to set up a state
     * machine waiting for the event to signal, then perform the action. Since the callback is done on the same
     * thread, the caller doesn't have to worry about thread safety.
     *
     * @param callback specifies the callback handler, null for removing previous callback handler.
     * @param callbackContext specifies the context object passing back to the callback handler.
     */
    public void setCallback(Callback callback, Object callbackContext)
    {
        setCallback(Thread.currentThread(), callback, callbackContext);
    }   //setCallback

    /**
     * This method is called to set the callback context object typically before the event is signaled. In some
     * scenarios, the callbackContext is set by the one who's signaling the event, not the one who calls setCallback.
     * Therefore, this method provides a way to do just that.
     *
     * @param callbackContext specifies the callback context object.
     */
    public void setCallbackContext(Object callbackContext)
    {
        this.callbackContext = callbackContext;
    }   //setCallbackContext

    /**
     * This method is called by a periodic thread when the thread has just been started and before it enters its
     * thread loop to register for event callback. When a callback handler is set for an event, the event is added
     * to the event list for the thread. The periodic thread will then periodically call checkForEventCallback
     * to check if any events in the list are signaled or canceled. When that happens, the callback will be performed
     * and the event will be removed from the event list.
     *
     * @return true if registration was successful, false if the thread has already registered an event list before.
     */
    public static boolean registerEventCallback()
    {
        final Thread thread = Thread.currentThread();
        boolean alreadyRegistered;

        synchronized (callbackEventListMap)
        {
            alreadyRegistered = callbackEventListMap.containsKey(thread);

            if (!alreadyRegistered)
            {
                callbackEventListMap.put(thread, new CallbackEventList());
                staticTracer.traceDebug(
                    moduleName, "Registering thread " + thread.getName() + " for event callback.");
            }
            else
            {
                staticTracer.traceWarn(moduleName, "Thread " + thread.getName() + " is already registered.");
                TrcDbgTrace.printThreadStack();
            }
        }

        return !alreadyRegistered;
    }   //registerEventCallback

    /**
     * This method is called by a periodic thread when the thread has exited its thread loop and before it is
     * terminated to unregister its thread from event callback.
     *
     * @return true if unregister is successful, false if the thread was never registered.
     */
    public static boolean unregisterEventCallback()
    {
        final Thread thread = Thread.currentThread();
        CallbackEventList callbackEventList;

        synchronized (callbackEventListMap)
        {
            callbackEventList = callbackEventListMap.remove(thread);
        }

        if (callbackEventList == null)
        {
            staticTracer.traceWarn(moduleName, "Thread " + thread.getName() + " was never registered.");
            TrcDbgTrace.printThreadStack();
        }
        else
        {
            staticTracer.traceDebug(
                moduleName, "Unregistering thread " + thread.getName() + " for event callback.");
        }

        return callbackEventList != null;
    }   //unregisterEventCallback

    /**
     * This method is called by a periodic thread in its thread loop to check if any events in the list are signaled
     * or canceled. When that happens, it performs the event callback on the periodic thread.
     */
    public static void performEventCallback()
    {
        final Thread thread = Thread.currentThread();
        CallbackEventList callbackEventList;

        synchronized (callbackEventListMap)
        {
            callbackEventList = callbackEventListMap.get(thread);
        }

        if (callbackEventList != null)
        {
            ArrayList<TrcEvent> callbackList = new ArrayList<>();

            synchronized (callbackEventList.listLock)
            {
//                // Use a list Iterator because it is fail-fast and allows removing entries while iterating the list.
//                Iterator<TrcEvent> listIterator = callbackEventList.iterator();
//                while (listIterator.hasNext())
                // Iterating the list backward so that removing an event will not affect iteration.
                for (int i = callbackEventList.eventList.size() - 1; i >= 0; i--)
                {
//                    TrcEvent event = listIterator.next();
                    TrcEvent event = callbackEventList.eventList.get(i);
                    if (event.isSignaled() || event.isCanceled())
                    {
                        callbackList.add(event);
//                        listIterator.remove();
                        callbackEventList.eventList.remove(i);
                    }
                }
            }

            for (TrcEvent event: callbackList)
            {
                staticTracer.traceDebug(
                    moduleName, "Doing event callback for " + event + " on thread " + thread.getName() + ".");
                Callback callback = event.callback;
                Object context = event.callbackContext;
                // Clear the callback stuff before doing the callback since the callback may reuse and chain to
                // another callback.
                event.callback = null;
                event.callbackContext = null;
                callback.notify(context);
            }
        }
        else
        {
            staticTracer.traceWarn(moduleName, thread.getName() + " was never registered.");
            TrcDbgTrace.printThreadStack();
        }
    }   //performEventCallback

}   //class TrcEvent
