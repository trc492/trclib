/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

/**
 * This interface defines methods for the subsystems to implement exclusive ownership support. A subsystem can be
 * accessed by multiple callers unaware of each other. Exclusive ownership can be acquired before access will be
 * granted. This will prevent other callers from interfering an in-progress operation by a different caller.
 */
public interface TrcExclusiveSubsystem
{
    /**
     * This class encapsulates all the parameters required to acquire and release exclusive ownership for the
     * operation.
     */
    class OwnershipParams
    {
        String owner;
        TrcEvent completionEvent;
        TrcEvent releaseOwnershipEvent;
        TrcDbgTrace msgTracer;

        OwnershipParams(String owner, TrcEvent completionEvent, TrcEvent releaseOwnershipEvent, TrcDbgTrace msgTracer)
        {
            this.owner = owner;
            this.completionEvent = completionEvent;
            this.releaseOwnershipEvent = releaseOwnershipEvent;
            this.msgTracer = msgTracer;
        }   //OwnershipParams

    }   // class OwnershipParams

    /**
     * This method checks if the caller has exclusive ownership of the subsystem. For backward compatibility with
     * older code that's not aware of subsystem exclusive ownership, the owner parameter can be null. If the
     * subsystem has no owner currently and the caller is not aware of exclusive ownership, the caller is considered
     * to have acquired ownership. This means the caller is allowed to proceed with controlling the subsystem but if
     * a new caller comes in and acquires the ownership of the subsystem while an operation is in progress, the
     * operation will be interrupted and preempted by the new caller's operation. Therefore, callers unaware of
     * exclusive ownership can start an operation on the subsystem but their operations are not guaranteed exclusive
     * ownership.
     *
     * @param owner specifies the ID string of the caller, can be null if caller is unaware of exclusive ownership.
     * @return true if caller has exclusive ownership of the subsystem, false otherwise.
     */
    default boolean hasOwnership(String owner)
    {
        return TrcOwnershipMgr.getInstance().hasOwnership(owner, this);
    }   //hasOwnership

    /**
     * This method returns the current owner of this subsystem.
     *
     * @return current owner, null if no owner.
     */
    default String getCurrentOwner()
    {
        return TrcOwnershipMgr.getInstance().getOwner(this);
    }   //getCurrentOwner

    /**
     * This method checks if the caller has exclusive ownership of the subsystem. If not, it throws an exception.
     * It throws an exception only if the caller is aware of exclusive ownership and the it doesn't currently own
     * the subsystem. If the caller is unaware of exclusive ownership and the subsystem is owned by somebody else,
     * it will just return false and not throw an exception. This is to ensure older code that's unaware of exclusive
     * ownership will not hit an unexpected exception and will just fail quietly.
     *
     * @param owner specifies the ID string of the caller, can be null if caller is unaware of exclusive ownership.
     * @return true if the caller currently owns the subsystem, false otherwise.
     * @throws IllegalStateException if caller is not the owner of the subsystem.
     */
    default boolean validateOwnership(String owner)
    {
        return TrcOwnershipMgr.getInstance().validateOwnership(owner, this);
    }   //validateOnwership

    /**
     * This method acquires exclusive ownership of the subsystem if it's not already owned by somebody else.
     *
     * @param owner specifies the ID string of the caller requesting ownership.
     * @return true if successfully acquired ownership, false otherwise.
     */
    default boolean acquireExclusiveAccess(String owner)
    {
        return TrcOwnershipMgr.getInstance().acquireOwnership(owner, this);
    }   //acquireExclusiveAccess

    /**
     * This method release exclusive ownership of the subsystem if the caller is indeed the owner.
     *
     * @param owner specifies the ID string of the caller releasing ownership.
     * @return true if successfully releasing ownership, false otherwise.
     */
    default boolean releaseExclusiveAccess(String owner)
    {
        return TrcOwnershipMgr.getInstance().releaseOwnership(owner, this);
    }   //releaseExclusiveAccess

    /**
     * This method release exclusive ownership of the subsystem regardless who owns it.
     *
     * @return true if successfully releasing ownership, false if no owner.
     */
    default boolean cancelExclusiveAccess()
    {
        return TrcOwnershipMgr.getInstance().cancelOwnership(this);
    }   //cancelExclusiveAccess

    /**
     * This method acquires exclusive access to the subsystem for the specified owner if the owner did not already
     * have ownership. If successfully acquired exclusive access, it will return an event that the owner must signal
     * when exclusive access is no longer needed. At that time, the exclusive access will be released for the owner.
     *
     * @param owner specifies the owner who wishes to acquire exclusive access, can be null if not requiring
     *        exclusive access in which case this call is a no-op.
     * @param completionEvent specifies the original event that will be signaled when the operation is completed.
     *        This event will be signaled for the owner when the exclusive access is released.
     * @return release ownership event that the caller must signal when exclusive access is no longer needed.
     *         Typically, it replaces the original completion event of the operation so that when the operation
     *         is completed, the caller will signal this event to release the exclusive access and it will then
     *         signal the original completion event for the caller.
     */
    default TrcEvent acquireOwnership(String owner, TrcEvent completionEvent, TrcDbgTrace msgTracer)
    {
        TrcEvent releaseOwnershipEvent = null;

        // Caller specifies an owner but has not acquired ownership, let's acquire ownership on its behalf.
        if (owner != null && !hasOwnership(owner) && acquireExclusiveAccess(owner))
        {
            if (msgTracer != null)
            {
                msgTracer.traceInfo(this.toString(), "Acquired ownership on behalf of " + owner);
            }
            releaseOwnershipEvent = new TrcEvent(this + ".releaseOwnership");
            OwnershipParams ownershipParams = new OwnershipParams(
                owner, completionEvent, releaseOwnershipEvent, msgTracer);
            releaseOwnershipEvent.setCallback(this::releaseOwnership, ownershipParams);
        }

        return releaseOwnershipEvent;
    }   //acquireOwnership

    /**
     * This method is an event callback to release exclusive ownership after the operation is completed or canceled.
     * It will also signal the original completion event for the owner.
     *
     * @param context specifies the releaseOwnership parameters.
     */
    default void releaseOwnership(Object context)
    {
        OwnershipParams ownershipParams = (OwnershipParams) context;

        releaseExclusiveAccess(ownershipParams.owner);
        if (ownershipParams.msgTracer != null)
        {
            ownershipParams.msgTracer.traceInfo(
                this.toString(), "Released ownership on behalf of " + ownershipParams.owner);
        }

        if (ownershipParams.completionEvent != null)
        {
            if (ownershipParams.releaseOwnershipEvent.isSignaled())
            {
                // Operation was completed successfully, indicate so in the completion event.
                ownershipParams.completionEvent.signal();
            }
            else
            {
                // Operation was canceled, indicate so in the completion event.
                ownershipParams.completionEvent.cancel();
            }

            if (ownershipParams.msgTracer != null)
            {
                ownershipParams.msgTracer.traceInfo(
                    this.toString(), "Signal completion event " + ownershipParams.completionEvent);
            }
        }
    }   //releaseOwnership

}   //interface TrcExclusiveSubsystem
