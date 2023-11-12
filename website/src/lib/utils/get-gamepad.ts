import { writable } from 'svelte/store'

export const gamepadIndex = writable<number | null>(null)

if (typeof window === 'object') {
  window.addEventListener('gamepadconnected', (e) => {
    gamepadIndex.set(e.gamepad.index)
  })
  window.addEventListener('gamepaddisconnected', () => {
    gamepadIndex.set(null)
  })
}
