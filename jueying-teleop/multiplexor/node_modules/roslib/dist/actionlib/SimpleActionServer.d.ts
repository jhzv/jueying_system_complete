import { EventEmitter } from 'eventemitter3';
import { default as Ros } from '../core/Ros.js';
import { default as Topic } from '../core/Topic.js';
/**
 * An actionlib action server client.
 *
 * Emits the following events:
 *  * 'goal' - Goal sent by action client.
 *  * 'cancel' - Action client has canceled the request.
 */
export default class SimpleActionServer extends EventEmitter<string | symbol, any> {
    /**
     * @param {Object} options
     * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
     * @param {string} options.serverName - The action server name, like '/fibonacci'.
     * @param {string} options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
     */
    constructor(options: {
        ros: Ros;
        serverName: string;
        actionName: string;
    });
    /** @type {{goal_id: {id: any, stamp: any}, goal: any} | null} */
    currentGoal: {
        goal_id: {
            id: any;
            stamp: any;
        };
        goal: any;
    } | null;
    /** @type {{goal_id: {id: any, stamp: any}, goal: any} | null} */
    nextGoal: {
        goal_id: {
            id: any;
            stamp: any;
        };
        goal: any;
    } | null;
    ros: Ros;
    serverName: string;
    actionName: string;
    feedbackPublisher: Topic<any>;
    resultPublisher: Topic<any>;
    statusMessage: {
        header: {
            stamp: {
                secs: number;
                nsecs: number;
            };
            frame_id: string;
        };
        /** @type {{goal_id: any, status: number}[]} */
        status_list: {
            goal_id: any;
            status: number;
        }[];
    };
    /**
     * Set action state to succeeded and return to client.
     *
     * @param {Object} result - The result to return to the client.
     */
    setSucceeded(result: any): void;
    /**
     * Set action state to aborted and return to client.
     *
     * @param {Object} result - The result to return to the client.
     */
    setAborted(result: any): void;
    /**
     * Send a feedback message.
     *
     * @param {Object} feedback - The feedback to send to the client.
     */
    sendFeedback(feedback: any): void;
    /**
     * Handle case where client requests preemption.
     */
    setPreempted(): void;
}
