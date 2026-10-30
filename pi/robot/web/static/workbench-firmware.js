// 与 pi/robot/device/capabilities.py 的 LATEST_MINOR 保持一致。
export const LATEST_MINOR = 51;

export function firmwareSince(firmware, minor) {
    const match = typeof firmware === 'string' && /^GC-BASE-0\.3\.(0|[1-9]\d*)$/.exec(firmware);
    if (!match) return false;
    const value = Number(match[1]);
    return minor <= value && value <= LATEST_MINOR;
}
