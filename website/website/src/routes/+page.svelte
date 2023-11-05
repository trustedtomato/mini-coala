<script lang="ts">
  import { processAxisValue } from '$lib/utils/process-axis-value'
  import { normalizeNumber } from '$lib/utils/normalize-number'
  import debug from 'debug'
  import { onMount } from 'svelte'

  const log = debug('app:main')

  // state
  let targetHeave = -0.2
  let heave = -0.2
  const minHeave = -10
  const maxHeave = 0

  // websocket connection
  let socket: WebSocket | null = null
  function initWebSocket() {
    socket = new WebSocket('ws://' + location.host + '/ros')
    socket.onopen = () => {
      log('WebSocket connected')
    }
    socket.onclose = () => {
      log('WebSocket disconnected')
      setTimeout(initWebSocket, 1000)
    }
    socket.onmessage = (e) => {
      log('WebSocket message', e.data)
    }
  }

  onMount(() => {
    initWebSocket()
    log('Init WebSocket')
  })

  // handle joystick
  $: normalizedHeave = 1 - normalizeNumber(heave, minHeave, maxHeave)
  $: normalizedTargetHeave = 1 - normalizeNumber(targetHeave, minHeave, maxHeave)

  let gamepadIndex: number | null = null
  function loop(): void {
    if (gamepadIndex !== null) {
      const gamepad = navigator.getGamepads()[gamepadIndex]
      const axes = gamepad.axes
      targetHeave += -processAxisValue(axes[1]) / 10
      targetHeave = Math.max(minHeave, Math.min(maxHeave, targetHeave))

      socket?.send(JSON.stringify({ heave: targetHeave }))
    }

    requestAnimationFrame(loop)
  }

  onMount(() => {
    window.addEventListener('gamepadconnected', (e) => {
      console.log(e.gamepad)
      gamepadIndex = e.gamepad.index
    })
    loop()
  })
</script>

<h1>COALA</h1>

Normalized heave: {normalizedTargetHeave.toFixed(2)}
