export const minPitch = -180
export const maxPitch = 180

export const minYaw = -180
export const maxYaw = 180

export const minRoll = -90
export const maxRoll = 90

export const pitchOffset = 90
export const rollOffset = 0
export const yawOffset = -90

export function getRad(deg: number): number {
  return (Math.PI * deg) / 180
}
