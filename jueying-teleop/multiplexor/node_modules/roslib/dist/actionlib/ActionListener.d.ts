import { EventEmitter } from 'eventemitter3';
import { default as Ros } from '../core/Ros.js';
/**
 * An actionlib action listener.
 *
 * Emits the following events:
 *  * 'status' - The status messages received from the action server.
 *  * 'feedback' - The feedback messages received from the action server.
 *  * 'result' - The result returned from the action server.
 *

 */
export default class ActionListener extends EventEmitter<string | symbol, any> {
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
    ros: Ros;
    serverName: string;
    actionName: string;
}
