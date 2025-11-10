import { EventEmitter } from 'eventemitter3';
import { default as Ros } from '../core/Ros.js';
import { default as Topic } from '../core/Topic.js';
/**
 * An actionlib action client.
 *
 * Emits the following events:
 *  * 'timeout' - If a timeout occurred while sending a goal.
 *  * 'status' - The status messages received from the action server.
 *  * 'feedback' - The feedback messages received from the action server.
 *  * 'result' - The result returned from the action server.
 *
 */
export default class ActionClient extends EventEmitter<string | symbol, any> {
    /**
     * @param {Object} options
     * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
     * @param {string} options.serverName - The action server name, like '/fibonacci'.
     * @param {string} options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
     * @param {number} [options.timeout] - The timeout length when connecting to the action server.
     * @param {boolean} [options.omitFeedback] - The flag to indicate whether to omit the feedback channel or not.
     * @param {boolean} [options.omitStatus] - The flag to indicate whether to omit the status channel or not.
     * @param {boolean} [options.omitResult] - The flag to indicate whether to omit the result channel or not.
     */
    constructor(options: {
        ros: Ros;
        serverName: string;
        actionName: string;
        timeout?: number | undefined;
        omitFeedback?: boolean | undefined;
        omitStatus?: boolean | undefined;
        omitResult?: boolean | undefined;
    });
    goals: {};
    /** flag to check if a status has been received */
    receivedStatus: boolean;
    ros: Ros;
    serverName: string;
    actionName: string;
    timeout: number | undefined;
    omitFeedback: boolean | undefined;
    omitStatus: boolean | undefined;
    omitResult: boolean | undefined;
    feedbackListener: Topic<any>;
    statusListener: Topic<any>;
    resultListener: Topic<any>;
    goalTopic: Topic<any>;
    cancelTopic: Topic<any>;
    /**
     * Cancel all goals associated with this ActionClient.
     */
    cancel(): void;
    /**
     * Unsubscribe and unadvertise all topics associated with this ActionClient.
     */
    dispose(): void;
}
