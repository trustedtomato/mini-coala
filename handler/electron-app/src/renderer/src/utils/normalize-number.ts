/** Normalize a number so that it's value will be between 0 and 1. */
export function normalizeNumber(value: number, min: number, max: number): number {
  return (value - min) / (max - min)
}
