<script lang="ts">
  import debug from 'debug'
  import { processAxisValue } from './utils/process-axis-value'
  import { normalizeNumber } from './utils/normalize-number'

  const log = debug('app:App')

  let targetHeave = -0.2
  let heave = -0.2
  const minHeave = -10
  const maxHeave = 0
  $: normalizedHeave = 1 - normalizeNumber(heave, minHeave, maxHeave)
  $: normalizedTargetHeave = 1 - normalizeNumber(targetHeave, minHeave, maxHeave)

  let gamepadIndex: number | null = null
  ;(function loop(): void {
    if (gamepadIndex !== null) {
      const gamepad = navigator.getGamepads()[gamepadIndex]
      const axes = gamepad.axes
      targetHeave += -processAxisValue(axes[1]) / 10
      targetHeave = Math.max(minHeave, Math.min(maxHeave, targetHeave))
    }

    // mock heave change
    heave += (targetHeave - heave) / 100

    requestAnimationFrame(loop)
  })()

  window.addEventListener('gamepadconnected', (e) => {
    gamepadIndex = e.gamepad.index
  })
</script>

<!-- Show heave with SVG -->
<svg viewBox="0 0 100 100" width="200">
  <line
    x1="0"
    y1={normalizedHeave * 100}
    x2="100"
    y2={normalizedHeave * 100}
    stroke="black"
    stroke-width="1"
  />
  <text x="0" y="10" font-size="10">
    {heave.toFixed(2)}
  </text>
  <!-- dashed red line -->
  <line
    x1="0"
    y1={normalizedTargetHeave * 100}
    x2="100"
    y2={normalizedTargetHeave * 100}
    stroke="red"
    stroke-width="1"
    stroke-dasharray="2"
  />
</svg>

<style>
  svg {
    border: 1px solid #aaa;
  }
</style>
