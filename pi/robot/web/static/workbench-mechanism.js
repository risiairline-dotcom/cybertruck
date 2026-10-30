import {initDevice} from './workbench-device.js?v=49.1';

/* 机械页仅接实车API；导航模型的刷新不参与本页状态或控制权。 */
let remote;
export function initMechanism() {
    const root = document.getElementById('mechanism-root');
    root.replaceChildren();
    remote = initDevice(root);
}
export function renderMechanism() {}
export function cancelMechHeld() {}
export async function stopMechHeld() {
    await remote?.release('page-leave');
}
export async function stopMechanism(source) {
    await remote?.stop(source);
}
