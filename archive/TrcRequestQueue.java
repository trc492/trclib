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

package trclib.archive;

import java.util.concurrent.LinkedBlockingQueue;

import trclib.timer.TrcTimer;

/**
 * This class implements a generic request queue that runs on its own thread. It allows the caller to add requests
 * to the end of the queue. The request thread will call the client to process the request asynchronously from the
 * head of the queue. When the request is completed, an optional event will be signaled as well as an optional
 * callback if provided.
 *
 * @param <R> specifies the type of the request.
 */
public class TrcRequestQueue<R>
{
    /**
     * This class implements a request entry. Typically, an entry will be put into a FIFO request queue so that each
     * entry will be processed in the order they came in.
     */
    public class RequestEntry
    {
        private final R request;
        private final TrcEvent notifyEvent;
        private final boolean repeat;
        private boolean canceled;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param request specifies the request.
         * @param event specifies the event to notify when the request is up for processing.
         * @param repeat specifies true to re-queue the request when completed.
         */
        public RequestEntry(R request, TrcEvent event, boolean repeat)
        {
            this.request = request;
            this.notifyEvent = event;
            this.repeat = repeat;
            this.canceled = false;
        }   //RequestEntry

        /**
         * This method retrieves the request object.
         *
         * @return request object.
         */
        public R getRequest()
        {
            return request;
        }   //getRequest

        /**
         * This method checks if the request entry is canceled.
         *
         * @return true if the request entry is canceled, false otherwise.
         */
        public boolean isCanceled()
        {
            return canceled;
        }   //isCanceled

        /**
         * This method returns the request info as a string.
         *
         * @return request info string.
         */
        @Override
        public String toString()
        {
            return "request=" + request + ", repeat=" + repeat + ", canceled=" + canceled;
        }   //toString

    }   //class RequestEntry

    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final LinkedBlockingQueue<RequestEntry> requestQueue;
    private volatile Thread requestThread = null;
    private boolean enabled = false;
    private RequestEntry priorityRequest = null;
    private boolean perfTracingEnabled = false;
    private double totalNanoTime = 0.0;
    private int totalRequests = 0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcRequestQueue(String instanceName)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        requestQueue = new LinkedBlockingQueue<>();
    }   //TrcRequestQueue

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
     * This method enables/disables the request queue. On enable, it creates the request thread to start processing
     * request entries in the queue. On disable, it shuts down the request thread and cancels all pending requests
     * still in the queue.
     *
     * @param enabled specifies true to enable request queue, false to disable.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            //
            // Enabling request queue, make sure the request queue is not already enabled.
            //
            if (requestThread == null)
            {
                requestThread = new Thread(this::requestTask, instanceName);
                requestThread.start();
                this.enabled = true;
            }
        }
        else if (requestThread != null)
        {
            //
            // Disabling request queue, make sure the request queue is indeed active.
            // The request queue may not be empty. So we need to signal termination but allow the request queue to
            // orderly shut down. If request queue is already disabled but the request thread is still active, it
            // means the thread is busy emptying its queue. So we don't need to double signal termination.
            //
            if (this.enabled)
            {
                this.enabled = false;
                requestThread.interrupt();
            }
        }
    }   //setEnabled

    /**
     * This method checks if the request queue is enabled.
     *
     * @return true if request queue is enabled, false if disabled.
     */
    public synchronized boolean isEnabled()
    {
        return enabled;
    }   //isEnabled

    /**
     * This method enables/disables performance report.
     *
     * @param enabled specifies true to enable performance tracing, false to disable.
     */
    public synchronized void setPerformanceTracingEnabled(boolean enabled)
    {
        perfTracingEnabled = enabled;
    }   //setPerformanceTracingEnabled

    /**
     * This method queues a request at the end of the request queue to be processed asynchronously on a thread.
     *
     * @param request specifies the request to be queued.
     * @param event specifies the event to notify when the request is up for processing.
     * @param repeat specifies true to re-queue the request when completed.
     * @return request entry added to the end of the queue. It can be used to cancel the request if it is still in
     *         queue.
     */
    public RequestEntry add(R request, TrcEvent event, boolean repeat)
    {
        tracer.traceDebug(instanceName, "request=" + request + ",repeat=" + repeat);
        RequestEntry entry = null;
        if (isEnabled())
        {
            entry = new RequestEntry(request, event, repeat);
            requestQueue.add(entry);
        }

        return entry;
    }   //add

    /**
     * This method adds the priority request to the head of the queue. It will be processed once the current active
     * request is done processing. If there is already an existing priority request pending, this request will not
     * be added to the queue and null is returned.
     *
     * @param request specifies the priority request.
     * @param event specifies the event to notify when the request is up for processing.
     * @return request entry added to the head of the queue. It can be used to cancel the request if it is still in
     *         queue. It may return null if the priority request failed to be added to the queue.
     */
    public synchronized RequestEntry addPriorityRequest(R request, TrcEvent event)
    {
        RequestEntry entry = null;

        tracer.traceDebug(instanceName, "request=" + request);
        if (priorityRequest == null)
        {
            entry = new RequestEntry(request, event, false);
            priorityRequest = entry;
        }

        return entry;
    }   //addPriorityRequest

    /**
     * This method cancels a request.
     *
     * @param entry specifies the request entry from add or addPriorityRequest to be canceled.
     * @return true if the request entry is found in the queue and canceled, false otherwise.
     */
    public synchronized boolean cancelRequest(RequestEntry entry)
    {
        boolean foundEntry;

        tracer.traceDebug(instanceName, "entry=" + entry);
        if (entry == priorityRequest)
        {
            priorityRequest = null;
            foundEntry = true;
        }
        else
        {
            foundEntry = requestQueue.contains(entry);
            if (foundEntry) requestQueue.remove(entry);
        }

        if (foundEntry)
        {
            entry.canceled = true;
        }

        return foundEntry;
    }   //cancelRequest

    /**
     * This method is called when the request queue thread is started. It processes all entries in the request queue
     * when they arrive. If the request queue is empty, the thread is blocked until a new request arrives. Therefore,
     * this thread only runs when there are requests in the queue. If this thread is interrupted, it will clean up
     * the request queue before exiting.
     */
    private void requestTask()
    {
        RequestEntry entry;

        tracer.traceDebug(instanceName, "RequestQueue starting...");
        while (!Thread.currentThread().isInterrupted())
        {
            synchronized (this)
            {
                if (priorityRequest != null)
                {
                    entry = priorityRequest;
                    priorityRequest = null;
                }
                else
                {
                    entry = null;
                }
            }

            try
            {
                if (entry == null)
                {
                    entry = requestQueue.take();
                }
                tracer.traceDebug(instanceName, "processing request " + entry);

                long startNanoTime = TrcTimer.getNanoTime();
                entry.notifyEvent.signal();
                long elapsedTime = TrcTimer.getNanoTime() - startNanoTime;
        
                totalNanoTime += elapsedTime;
                totalRequests++;
        
                if (perfTracingEnabled)
                {
                    tracer.traceInfo(
                        instanceName,
                        "Average request process time=%.6f sec", totalNanoTime/totalRequests/1000000000.0);
                }
        
                if (entry.repeat)
                {
                    //
                    // This is a repeat request, add it back to the tail of the queue.
                    //
                    requestQueue.add(entry);
                }
            }
            catch (InterruptedException e)
            {
                tracer.traceDebug(instanceName, "Terminating RequestQueue.");
                break;
            }
        }
        //
        // The thread is terminating, empty the queue before exiting and make sure nobody is trying to start another
        // thread until this one has exited.
        //
        synchronized (this)
        {
            while ((entry = requestQueue.poll()) != null)
            {
                tracer.traceDebug(instanceName, "Canceling request " + entry);
                cancelRequest(entry);
            }

            requestThread = null;
        }
        tracer.traceDebug(instanceName, "RequestQueue is terminated.");
    }   //requestTask

}   //class TrcRequestQueue
