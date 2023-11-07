<script lang="ts">
  import '../app.css'
  import { processAxisValue } from '$lib/utils/process-axis-value'
  import { normalizeNumber } from '$lib/utils/normalize-number'
  import debug from 'debug'
  import { onMount } from 'svelte'
  import SubcLogo from '$lib/components/subc-logo.svelte'
  import Coala_3d from '$lib/components/sections/coala-3d.svelte'
  import Bars from '$lib/components/sections/bars.svelte'
  import Graph from '$lib/components/sections/graph.svelte'
  import AauLogo from '$lib/components/aau-logo.svelte'
  import Section from '$lib/components/sections/section.svelte'

  const log = debug('app:main')

  // state
  let targetHeave = -0.2
  let heave = -0.2
  const minHeave = -10
  const maxHeave = 0

  // websocket connection
  let socket: WebSocket | null = null
  let socketConnected = false
  function initWebSocket() {
    socket = new WebSocket('ws://' + location.host + '/ros')
    socket.onopen = () => {
      log('WebSocket connected')
      socketConnected = true
    }
    socket.onclose = () => {
      log('WebSocket disconnected')
      socketConnected = false
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
    const gamepad = gamepadIndex !== null ? navigator.getGamepads()[gamepadIndex] : null
    if (gamepad) {
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

<svelte:head>
  <title>Mini COALA Controller</title>
</svelte:head>

<header class="bg-x-blue">
  <div class="container">
    <div class="flex py-2 items-center gap-4 text-x-white font-medium">
      <div class="h-8">
        <SubcLogo />
      </div>
      <div>Mini COALA Controller</div>
    </div>
  </div>
</header>

<div class="container">
  <div class="flex gap-8">
    <div class="basis-52 shrink-0 grow-0">
      <Coala_3d header="Target pitch, yaw" />
      <Coala_3d header="Measured pitch, yaw" />
      <Section>
        <span slot="header">Connectivity</span>
        <div class="my-2">
          Network:
          {#if socketConnected}
            <span class="text-x-green">✓&nbsp;Connected</span>
          {:else}
            <span class="text-x-red">
              🞫&nbsp;Disconnected
              <span class="text-sm">(reconnecting...)</span>
            </span>
          {/if}
        </div>
        <div>
          Gamepad:
          {#if gamepadIndex !== null}
            <span class="text-x-green">✓&nbsp;Connected</span>
          {:else}
            <span class="text-x-red">
              🞫&nbsp;Disconnected
              <span class="text-sm"> (try pressing something on the gamepad) </span>
            </span>
          {/if}
        </div>
      </Section>
    </div>
    <div class="basis-52 shrink-0 grow-0">
      <Bars header="Heave: target, measured" barValues={[0.2, 0.5]} />
      <Bars header="Pitch: target, measured" barValues={[0.2, 0.5]} />
      <Bars header="Yaw: target, measured" barValues={[0.2, 0.5]} />
      <Bars header="Target surge velocity" barValues={[0.2, 0.5]} />
      <Bars header="Thruster strenghts" barValues={Array.from({ length: 10 }, (_, i) => i / 10)} />
    </div>
    <div class="basis-0 grow">
      <Graph header="Heave history" />
      <Graph header="Pitch history" />
      <Graph header="Yaw history" />
    </div>
  </div>
</div>

<footer class="bg-x-blue">
  <div class="container">
    <div class="flex py-2 items-center gap-4 text-x-white font-medium">
      <div class="h-8 shrink-0 flex gap-4">
        <SubcLogo />
        <AauLogo />
      </div>
      <div class="ml-auto font-normal text-sm leading-tight">
        Made by Norbert Pap, Tamas Halasi and Tue Jensen as part of their 5th semester project
        studying Applied Industrial Electronics BSc at Aalborg University, Esbjerg, in cooperation
        with SubC.
      </div>
    </div>
  </div>
</footer>
