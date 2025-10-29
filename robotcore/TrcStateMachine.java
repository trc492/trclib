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

import trclib.timer.TrcTimer;

/**
 * This class implements an event driven state machine. The caller can add multiple events for the state machine
 * to monitor. If one or more events are signaled, the state machine will automatically advance to the specified
 * next state.
 *
 * @param <T> specifies the State enum type that list all possible states.
 */
public class TrcStateMachine<T>
{
    private final String instanceName;
    private final ArrayList<TrcEvent> eventList = new ArrayList<>();
    private T currState = null;
    private T nextState = null;
    private boolean enabled = false;
    private boolean ready = false;
    private boolean expired = false;
    private double expiredTime = 0.0;
    private boolean waitForAllEvents = false;

    /**
     * Constructor: Creates an instance of the state machine with the given name.
     *
     * @param instanceName specifies the instance name of the state machine.
     */
    public TrcStateMachine(String instanceName)
    {
        this.instanceName = instanceName;
    }   //TrcStateMachine

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method starts the state machine with the given starting state and puts it in ready mode.
     *
     * @param state specifies the starting state.
     */
    public void start(T state)
    {
        eventList.clear();
        currState = state;
        nextState = state;
        enabled = true;
        ready = true;
        expired = false;
        expiredTime = 0.0;
        waitForAllEvents = false;
    }   //start

    /**
     * This method stops the state machine by disabling it.
     */
    public void stop()
    {
        eventList.clear();
        currState = null;
        nextState = null;
        enabled = false;
        ready = false;
        expired = false;
        expiredTime = 0.0;
        waitForAllEvents = false;
    }   //stop

    /**
     * This method returns the current state of the state machine.
     *
     * @return current state of the state machine.
     */
    public T getState()
    {
        return currState;
    }   //getState

    /**
     * This method returns the next state of the state machine.
     *
     * @return next state of the state machine.
     */
    public T getNextState()
    {
        return nextState;
    }   //getNextState

    /**
     * This method checks whether the state machine is ready. If so, it returns the current state. If the state
     * machine is not ready, it returns null.
     *
     * @return current state of the state machine, null if state machine is not ready.
     */
    public T checkReadyAndGetState()
    {
        T state = null;

        if (isReady())
        {
            state = getState();
        }

        return state;
    }   //checkReadyGetState

    /**
     * This method sets the current state of the state machine.
     *
     * @param state specifies the state to set the state machine to.
     */
    public void setState(T state)
    {
        currState = state;
    }   //setState

    /**
     * This method checks if the state machine is enabled.
     *
     * @return true if state machine is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        return enabled;
    }   //isEnabled

    /**
     * This method checks if the state machine is in ready mode. If not, it will enumerate all the events it is
     * monitoring and make sure if any or all of them are signaled as the condition for putting the state machine
     * back in ready mode.
     *
     * @return true if the state machine is in ready mode, false otherwise.
     */
    public boolean isReady()
    {
        //
        // If the state machine is enabled but not ready, check all events if the state machine should be put back
        // in ready mode.
        //
        if (enabled && !ready)
        {
            //
            // If a timeout was specifies and we have past the timeout time, we will put the state machine back to
            // ready mode but indicate the timeout had expired.
            //
            if (expiredTime > 0.0 && TrcTimer.getCurrentTime() >= expiredTime)
            {
                expiredTime = 0.0;
                ready = true;
                expired = true;
            }
            else
            {
                //
                // Count the number of signaled events.
                //
                int count = 0;
                for (TrcEvent event: eventList)
                {
                    if (event.isSignaled() || event.isCanceled())
                    {
                        count++;
                    }
                }

                //
                // If waitForAllEvents is true, the number of signaled events must equal to the size of the event
                // list (i.e. all events have signaled). If waitForAllEvents is false, then we just need a non-zero
                // count in order to put the state machine back to ready mode.
                //
                if (!waitForAllEvents && count > 0 || waitForAllEvents && count == eventList.size())
                {
                    ready = true;
                }
            }

            //
            // If we put the state machine back to ready mode, we need to clear all events and the event list to
            // monitor. Then we move the state from the current state to the next state.
            //
            if (ready)
            {
                eventList.clear();
                currState = nextState;
            }
        }

        return enabled && ready;
    }   //isReady

    /**
     * This method clears the event list preparing for adding multiple event to the list.
     */
    public void clearEventList()
    {
        eventList.clear();
    }   //clearEventList

    /**
     * This method checks if timeout has happened on waiting for event(s).
     *
     * @return true if a timeout was set and expired, false otherwise.
     */
    public boolean isTimedout()
    {
        return expired;
    }   //isTimedout

    /**
     * This method adds an event to the event list to be monitored.
     *
     * @param event specifies the vent to be added to the list.
     */
    public void addEvent(TrcEvent event)
    {
        //
        // Only add to the list if the given event is not already in the list.
        //
        if (!eventList.contains(event))
        {
            eventList.add(event);
        }
    }   //addEvent

    /**
     * This method puts the state machine into not ready mode and starts monitoring the events in the list. If
     * waitForAllEvents is false and any event on the list is signaled or waitForAllEvents is true and all events
     * are signaled, the state machine will be put back to ready mode and it will automatically advance to the
     * given next state. If timeout is non-zero, the state machine will be put back to ready mode after timeout
     * has expired even though the required event(s) have not been signaled.
     *
     * @param nextState specifies the next state when the state machine becomes ready.
     * @param clearEvents specifies true to clear all events.
     * @param waitForAllEvents specifies true if all events must be signaled for the state machine to go ready.
     *                         If false, any signaled event will cause the state machine to go ready.
     * @param timeout specifies a timeout value. A zero value means there is no timeout.
     */
    public void waitForEvents(T nextState, boolean clearEvents, boolean waitForAllEvents, double timeout)
    {
        this.nextState = nextState;
        this.expiredTime = timeout;
        if (timeout > 0.0)
        {
            this.expiredTime += TrcTimer.getCurrentTime();
        }
        this.waitForAllEvents = waitForAllEvents;
        ready = false;
        if (clearEvents)
        {
            clearAllEvents();
        }
    }   //waitForEvents

    /**
     * This method puts the state machine into not ready mode and starts monitoring the events in the list. If
     * waitForAllEvents is false and any event on the list is signaled or waitForAllEvents is true and all events
     * are signaled, the state machine will be put back to ready mode and it will automatically advance to the
     * given next state.
     *
     * @param nextState specifies the next state when the state machine becomes ready.
     * @param clearEvents specifies true to clear all events.
     * @param waitForAllEvents specifies true if all events must be signaled for the state machine to go ready.
     *                         If false, any signaled event will cause the state machine to go ready.
     */
    public void waitForEvents(T nextState, boolean clearEvents, boolean waitForAllEvents)
    {
        waitForEvents(nextState, clearEvents, waitForAllEvents, 0.0);
    }   //waitForEvents

    /**
     * This method puts the state machine into not ready mode and starts monitoring the events in the list. If
     * any event on the list is signaled, the state machine will be put back to ready mode and it will automatically
     * advance to the given next state. If timeout is non-zero, the state machine will be put back to ready mode
     * after timeout has expired even though the required event(s) have not been signaled.
     *
     * @param nextState specifies the next state when the state machine becomes ready.
     * @param waitForAllEvents specifies true if all events must be signaled for the state machine to go ready.
     *                         If false, any signaled event will cause the state machine to go ready.
     * @param timeout specifies a timeout value. A zero value means there is no timeout.
     */
    public void waitForEvents(T nextState, boolean waitForAllEvents, double timeout)
    {
        waitForEvents(nextState, true, waitForAllEvents, timeout);
    }   //waitForEvents

    /**
     * This method puts the state machine into not ready mode and starts monitoring the events in the list. If
     * any event on the list is signaled, the state machine will be put back to ready mode and it will automatically
     * advance to the given next state. If timeout is non-zero, the state machine will be put back to ready mode
     * after timeout has expired even though the required event(s) have not been signaled.
     *
     * @param nextState specifies the next state when the state machine becomes ready.
     * @param waitForAllEvents specifies true if all events must be signaled for the state machine to go ready.
     *                         If false, any signaled event will cause the state machine to go ready.
     */
    public void waitForEvents(T nextState, boolean waitForAllEvents)
    {
        waitForEvents(nextState, true, waitForAllEvents, 0.0);
    }   //waitForEvents

    /**
     * This method puts the state machine into not ready mode and starts monitoring the events in the list. If
     * any event on the list is signaled, the state machine will be put back to ready mode and it will automatically
     * advance to the given next state. If timeout is non-zero, the state machine will be put back to ready mode
     * after timeout has expired even though the required event(s) have not been signaled.
     *
     * @param nextState specifies the next state when the state machine becomes ready.
     * @param timeout specifies a timeout value. A zero value means there is no timeout.
     */
    public void waitForEvents(T nextState, double timeout)
    {
        waitForEvents(nextState, true, false, timeout);
    }   //waitForEvents

    /**
     * This method puts the state machine into not ready mode and starts monitoring the events in the list. If any
     * event on the list is signaled, the state machine will be put back to ready mode and it will automatically
     * advance to the given next state.
     *
     * @param nextState specifies the next state when the state machine becomes ready.
     */
    public void waitForEvents(T nextState)
    {
        waitForEvents(nextState, true, false, 0.0);
    }   //waitForEvents

    /**
     * This method puts the state machine into not ready mode and starts monitoring a single event. If the event
     * is signaled, the state machine will be put back to ready mode and it will automatically advance to the given
     * next state. If timeout is non-zero, the state machine will be put back to ready mode after timeout has expired
     * even though the required event have not been signaled.
     *
     * @param event specifies the event to wait for.
     * @param nextState specifies the next state when the state machine becomes ready.
     * @param clearEvent specifies true to clear the event.
     * @param timeout specifies a timeout value. A zero value means there is no timeout.
     */
    public void waitForSingleEvent(TrcEvent event, T nextState, boolean clearEvent, double timeout)
    {
        eventList.clear();
        addEvent(event);
        waitForEvents(nextState, clearEvent, false, timeout);
    }   //waitForSingleEvent

    /**
     * This method puts the state machine into not ready mode and starts monitoring a single event. If the event
     * is signaled, the state machine will be put back to ready mode and it will automatically advance to the given
     * next state. If timeout is non-zero, the state machine will be put back to ready mode after timeout has expired
     * even though the required event have not been signaled.
     *
     * @param event specifies the event to wait for.
     * @param nextState specifies the next state when the state machine becomes ready.
     * @param timeout specifies a timeout value. A zero value means there is no timeout.
     */
    public void waitForSingleEvent(TrcEvent event, T nextState, double timeout)
    {
        eventList.clear();
        addEvent(event);
        waitForEvents(nextState, true, false, timeout);
    }   //waitForSingleEvent

    /**
     * This method puts the state machine into not ready mode and starts monitoring a single event. If the event
     * is signaled, the state machine will be put back to ready mode and it will automatically advance to the given
     * next state. If timeout is non-zero, the state machine will be put back to ready mode after timeout has expired
     * even though the required event have not been signaled.
     *
     * @param event specifies the event to wait for.
     * @param nextState specifies the next state when the state machine becomes ready.
     */
    public void waitForSingleEvent(TrcEvent event, T nextState)
    {
        eventList.clear();
        addEvent(event);
        waitForEvents(nextState, true, false, 0.0);
    }   //waitForSingleEvent

    /**
     * This method clears the signaled state of all the events in the list.
     */
    private void clearAllEvents()
    {
        for (int i = 0; i < eventList.size(); i++)
        {
            TrcEvent event = eventList.get(i);
            event.clear();
        }
    }   //clearAllEvents

}   //class TrcStateMachine
