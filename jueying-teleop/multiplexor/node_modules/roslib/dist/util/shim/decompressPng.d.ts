/**
 * @fileOverview
 * @author Graeme Yeates - github.com/megawac
 */
/**
 * @callback decompressPngCallback
 * @param data - The uncompressed data.
 */
/**
 * If a message was compressed as a PNG image (a compression hack since
 * gzipping over WebSockets * is not supported yet), this function places the
 * "image" in a canvas element then decodes the * "image" as a Base64 string.
 *
 * @private
 * @param data - An object containing the PNG data.
 * @param {decompressPngCallback} callback - Function with the following params:
 */
export default function decompressPng(data: any, callback: decompressPngCallback): void;
export type decompressPngCallback = (data: any) => any;
