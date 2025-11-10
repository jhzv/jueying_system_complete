import { EventEmitter } from 'eventemitter3';
import { default as ActionClient } from './ActionClient.js';
/**
 * An actionlib goal that is associated with an action server.
 *
 * Emits the following events:
 *  * 'timeout' - If a timeout occurred while sending a goal.
 */
export default class Goal extends EventEmitter<string | symbol, any> {
    /**
     * @param {Object} options
     * @param {ActionClient} options.actionClient - The ROSLIB.ActionClient to use with this goal.
     * @param {Object} options.goalMessage - The JSON object containing the goal for the action server.
     */
    constructor(options: {
        actionClient: ActionClient;
        goalMessage: any;
    });
    isFinished: boolean;
    status: undefined;
    result: undefined;
    feedback: undefined;
    goalID: string;
    actionClient: ActionClient;
    goalMessage: {
        goal_id: {
            stamp: {
                secs: number;
                nsecs: number;
            };
            id: string;
        };
        goal: any;
    };
    /**
     * Send the goal to the action server.
     *
     * @param {number} [timeout] - A timeout length for the goal's result.
     */
    send(timeout?: number): void;
    /**
     * Cancel the current goal.
     */
    cancel(): void;
}
