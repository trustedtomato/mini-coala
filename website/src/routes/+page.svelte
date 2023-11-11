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
  import type { ChartDataset } from 'chart.js'

  const log = debug('app:main')

  // state
  let heave = -0.2
  let targetHeave = -0.2
  const minHeave = -10
  const maxHeave = 0
  let pitch = 0
  let targetPitch = 0
  const minPitch = -Math.PI / 2
  const maxPitch = Math.PI / 2
  let yaw = 0
  let targetYaw = 0
  const minYaw = -Math.PI / 2
  const maxYaw = Math.PI / 2
  let targetSurgeVelocity = 0
  let thrusterStrengths = Array.from({ length: 10 }, () => 0)

  type Dataset = {
    x: number
    y: number
    time: number
  }[]

  const heaveHistory: Dataset = Array.from({ length: 100 }, () => ({
    x: 0,
    y: 0,
    time: 0
  }))

  const pitchHistory: Dataset = Array.from({ length: 100 }, () => ({
    x: 0,
    y: 0,
    time: 0
  }))

  const yawHistory: Dataset = Array.from({ length: 100 }, () => ({
    x: 0,
    y: 0,
    time: 0
  }))

  function pushToDataset(value: number, history: Dataset) {
    const currentTime = performance.now()
    // only push if the last push was more than 100ms ago
    if (currentTime < history[history.length - 1].time + 50) {
      return
    }
    history.push({
      time: currentTime,
      x: 0,
      y: value
    })
    // normalize x values so that the current value is at 0 and the oldest value is at -10
    for (let i = 0; i < history.length; i++) {
      history[i].x = (normalizeNumber(history[i].time, currentTime - 10000, currentTime) - 1) * 10
    }
    while (history[0].x < -10) {
      history.shift()
    }
  }

  $: pushToDataset(heave, heaveHistory)
  $: pushToDataset(pitch, pitchHistory)
  $: pushToDataset(yaw, yawHistory)

  const heaveDatasets: ChartDataset<'line'>[] = [
    {
      label: 'Heave',
      data: heaveHistory,
      borderColor: '#C01633'
    }
  ]
  const pitchDatasets: ChartDataset<'line'>[] = [
    {
      label: 'Pitch',
      data: pitchHistory,
      borderColor: '#C01633'
    }
  ]
  const yawDatasets: ChartDataset<'line'>[] = [
    {
      label: 'Yaw',
      data: yawHistory,
      borderColor: '#C01633'
    }
  ]

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

    // fake data change for testing
    let animationFrameRequest: number
    function loop() {
      heave += 0.01
      if (heave > maxHeave) {
        heave = minHeave
      }
      pitch += 0.01
      if (pitch > maxPitch) {
        pitch = minPitch
      }
      yaw += 0.01
      if (yaw > maxYaw) {
        yaw = minYaw
      }
      thrusterStrengths = Array.from({ length: 10 }, () => Math.random())
      animationFrameRequest = requestAnimationFrame(loop)
    }
    loop()
    return () => {
      cancelAnimationFrame(animationFrameRequest)
    }
  })

  // handle joystick
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
      <Coala_3d header="Target pitch, yaw" pitch={targetPitch} yaw={targetYaw} />
      <Coala_3d header="Measured pitch, yaw" {pitch} {yaw} />
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
      <Bars
        header="Heave: target, measured"
        minValue={minHeave}
        maxValue={maxHeave}
        barValues={[targetHeave, heave]}
      />
      <Bars
        header="Pitch: target, measured"
        minValue={minPitch}
        maxValue={maxPitch}
        barValues={[targetPitch, pitch]}
      />
      <Bars
        header="Yaw: target, measured"
        minValue={minYaw}
        maxValue={maxYaw}
        barValues={[targetYaw, yaw]}
      />
      <Bars header="Target surge velocity" barValues={[targetSurgeVelocity]} />
      <Bars header="Thruster strenghts" barValues={thrusterStrengths} />
    </div>
    <div class="basis-0 grow">
      <Graph
        header="Heave history"
        datasets={heaveDatasets}
        ySuggestedMin={minHeave}
        ySuggestedMax={maxHeave}
        yAxisLabel="Heave (m)"
      />
      <Graph
        header="Pitch history"
        datasets={pitchDatasets}
        ySuggestedMin={minPitch}
        ySuggestedMax={maxPitch}
        yAxisLabel="Pitch (rad)"
      />
      <Graph
        header="Yaw history"
        datasets={yawDatasets}
        ySuggestedMin={minYaw}
        ySuggestedMax={maxYaw}
        yAxisLabel="Yaw (rad)"
      />
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
