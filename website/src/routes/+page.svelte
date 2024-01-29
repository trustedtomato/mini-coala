<script lang="ts">
  import '../app.css'
  import { processAxisValue } from '$lib/utils/process-axis-value'
  import { minPitch, maxPitch, minRoll, maxRoll, minYaw, maxYaw } from '$lib/utils/get-rad'
  import { gamepadIndex } from '$lib/utils/get-gamepad'
  import { last } from '$lib/utils/last'
  import debug from 'debug'
  import { onMount } from 'svelte'
  import SubcLogo from '$lib/components/subc-logo.svelte'
  import Coala_3d from '$lib/components/sections/coala-3d.svelte'
  import Bars from '$lib/components/sections/bars.svelte'
  import Graph from '$lib/components/sections/history-graph.svelte'
  import AauLogo from '$lib/components/aau-logo.svelte'
  import Section from '$lib/components/sections/section.svelte'
  import type { ChartDataset } from 'chart.js'
  import { limit } from '$lib/utils/limit'
  import { zipTwo } from '$lib/utils/zip-two'

  const log = debug('app:main')

  // state
  let heave = -0.2
  let targetHeave = -0.2
  const minHeave = -10
  const maxHeave = 0
  let pitch = 0
  let targetPitch = 0
  let roll = 0
  let targetRoll = 0
  let yaw = 0
  let targetYaw = 0
  let targetSurgeVelocity = 0
  let thrusterStrengths = Array.from({ length: 10 }, () => 0)
  const minThrusterStrength = -1
  const maxThrusterStrength = 1

  const colors = [
    '#C01633',
    '#7784F7',
    '#F7D778',
    '#F778D1',
    '#78F7D7',
    '#78D7F7',
    '#78F7B4',
    '#F7B478',
    '#F77878',
    '#F77878'
  ]
  function zipLabelsWithBordercolors(labels: string[]): { borderColor: string; label: string }[] {
    return Array.from(zipTwo(colors, labels), ([borderColor, label]) => ({
      borderColor,
      label
    }))
  }

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
      // console.log(e.data)
      const data = JSON.parse(e.data) as { type: string; data: any }
      switch (data.type) {
        case 'motor':
          thrusterStrengths = data.data
          break
        case 'heave':
          heave = data.data
          break
        case 'imu':
          pitch = data.data.pitch
          yaw = data.data.yaw
          roll = data.data.roll
          break
      }
    }
  }

  onMount(() => {
    initWebSocket()
    log('Init WebSocket')

    // fake data change for testing
    /*
    let animationFrameRequest: number
    function loop() {
      heave += (targetHeave - heave) * 0.1
      yaw += (targetYaw - yaw) * 0.1
      pitch += 1
      if (pitch > maxPitch) {
        pitch = minPitch
      }
      thrusterStrengths = Array.from({ length: 10 }, () => Math.random())
      animationFrameRequest = requestAnimationFrame(loop)
    }
    loop()
    return () => {
      cancelAnimationFrame(animationFrameRequest)
    }
    */
  })

  // handle joystick
  function getGamepadState(gamepad: Gamepad) {
    const axes = gamepad.axes
    const nitro = !gamepad.buttons[7].pressed ? 0.1 : 1
    const a = gamepad.buttons[0]
    const b = gamepad.buttons[1]
    const x = gamepad.buttons[2]
    const y = gamepad.buttons[3]
    const arrowY = gamepad.buttons[12].value - gamepad.buttons[13].value
    return {
      deltaTargetHeave: (arrowY * nitro) / (maxHeave - minHeave),
      deltaTargetYaw: -(processAxisValue(axes[2]) * nitro) / (maxYaw - minYaw),
      targetSurgeVelocity: -processAxisValue(axes[1]) * nitro
    }
  }

  onMount(() => {
    log('Mounted!', $gamepadIndex)

    let animationFrameRequest: number | undefined
    function loop(): void {
      const gamepad = $gamepadIndex !== null ? navigator.getGamepads()[$gamepadIndex] : null
      if (gamepad) {
        const newState = getGamepadState(gamepad)

        targetHeave += newState.deltaTargetHeave
        targetHeave = limit(targetHeave, minHeave, maxHeave)
        targetYaw += newState.deltaTargetYaw
        // if target yaw is outside the range, rotate it back to the range
        if (targetYaw < minYaw) {
          targetYaw += Math.PI * 2
        } else if (targetYaw > maxYaw) {
          targetYaw -= Math.PI * 2
        }
        targetSurgeVelocity = newState.targetSurgeVelocity

        try {
          socket?.send(JSON.stringify({ targetHeave, targetYaw, targetSurgeVelocity }))
        } catch (e) {
          log('Error sending data to WebSocket', e)
        }
      }

      animationFrameRequest = requestAnimationFrame(loop)
    }
    loop()
    return () => {
      cancelAnimationFrame(animationFrameRequest!)
      if (socket) {
        socket.onopen = null
        socket.onclose = null
        socket.onmessage = null
        socket.close()
      }
    }
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
      <Coala_3d header="Target orientation" pitch={targetPitch} yaw={targetYaw} roll={targetRoll} />
      <Coala_3d header="Measured orientation" {pitch} {yaw} {roll} />
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
          {#if $gamepadIndex !== null}
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
        header="Roll: target, measured"
        minValue={minRoll}
        maxValue={maxRoll}
        barValues={[targetRoll, roll]}
      />
      <Bars
        header="Yaw: target, measured"
        minValue={minYaw}
        maxValue={maxYaw}
        barValues={[targetYaw, yaw]}
      />
      <Bars
        header="Target surge velocity"
        minValue={-1}
        maxValue={1}
        barValues={[targetSurgeVelocity]}
      />
      <Bars
        header="Thruster strenghts"
        minValue={minThrusterStrength}
        maxValue={maxThrusterStrength}
        barValues={thrusterStrengths}
      />
    </div>
    <div class="basis-0 grow bigDesktop:grid bigDesktop:grid-cols-2 bigDesktop:gap-x-8 min-w-0">
      <Graph
        header="Heave history"
        datasetConfigs={zipLabelsWithBordercolors(['Target', 'Measured'])}
        currentValues={[targetHeave, heave]}
        yMin={minHeave}
        yMax={maxHeave}
        yAxisLabel="Heave (m)"
      />
      <Graph
        header="Pitch history"
        datasetConfigs={zipLabelsWithBordercolors(['Target', 'Measured'])}
        currentValues={[targetPitch, pitch]}
        yMin={minPitch}
        yMax={maxPitch}
        yAxisLabel="Pitch (rad)"
      />
      <Graph
        header="Roll history"
        datasetConfigs={zipLabelsWithBordercolors(['Target', 'Measured'])}
        currentValues={[targetRoll, roll]}
        yMin={minRoll}
        yMax={maxRoll}
        yAxisLabel="Pitch (rad)"
      />
      <Graph
        header="Yaw history"
        datasetConfigs={zipLabelsWithBordercolors(['Target', 'Measured'])}
        currentValues={[targetYaw, yaw]}
        yMin={minYaw}
        yMax={maxYaw}
        yAxisLabel="Yaw (rad)"
      />
      <div class="col-span-2">
        <Graph
          header="Thruster history"
          datasetConfigs={zipLabelsWithBordercolors(
            Array.from({ length: 10 }, (_, i) => `${i + 1}`)
          ).map((d) => ({
            ...d,
            hidden: true
          }))}
          currentValues={thrusterStrengths}
          yMin={minThrusterStrength}
          yMax={maxThrusterStrength}
          yAxisLabel="Yaw (rad)"
        />
      </div>
    </div>
  </div>
</div>

<footer class="bg-x-blue mt-8">
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
