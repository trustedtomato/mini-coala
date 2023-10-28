const epsilon = 0.1

export function processAxisValue(value: number): number {
  if (Math.abs(value) < epsilon) {
    return 0
  }
  return value
}
