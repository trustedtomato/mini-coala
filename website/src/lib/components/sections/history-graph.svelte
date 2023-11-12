<script lang="ts">
  import { Chart, registerables, type ChartDataset } from 'chart.js'
  import Section from './section.svelte'
  import { onMount } from 'svelte'
  import { last } from '$lib/utils/last'
  import { normalizeNumber } from '$lib/utils/normalize-number'
  import { zipTwo } from '$lib/utils/zip-two'

  export let header = 'Chart'
  export let subtitle = ''
  export let yAxisLabel = 'Heave (m)'
  export let yMin = 0
  export let yMax = 1

  export let datasetConfigs: {
    label: string
    borderColor: string
  }[] = []

  export let currentValues = [] as number[]

  // added some defaults as per:
  // https://www.chartjs.org/docs/latest/general/performance.html
  const datasetDefaults: Partial<ChartDataset<'line'>> = {
    borderCapStyle: 'round',
    borderJoinStyle: 'round',
    showLine: true,
    pointRadius: 0,
    spanGaps: true
  }

  const datasets = datasetConfigs.map((dataset) => ({
    ...datasetDefaults,
    ...dataset,
    data: [] as { time: number; x: number; y: number }[]
  }))

  Chart.register(...registerables)

  let canvas: HTMLCanvasElement | undefined

  onMount(() => {
    const ctx = canvas!.getContext('2d')!
    const chart = new Chart(ctx, {
      type: 'line',
      data: {
        datasets
      },
      options: {
        responsive: true,
        normalized: true,
        animation: false,
        parsing: false,
        scales: {
          x: {
            type: 'linear',
            title: {
              display: true,
              text: 'Time (s)'
            },
            min: -10,
            max: 0,
            ticks: {
              minRotation: 0,
              maxRotation: 0,
              stepSize: 1,
              sampleSize: 1
            }
          },
          y: {
            type: 'linear',
            title: {
              display: true,
              text: yAxisLabel
            },
            min: yMin,
            max: yMax
          }
        },
        plugins: {
          legend: {
            display: false
          },
          tooltip: {
            enabled: false
          }
        }
      }
    })

    let animationFrameRequest: number | undefined
    function animate() {
      animationFrameRequest = requestAnimationFrame(animate)
      const currentTime = performance.now()
      // only push if the last push was more than 100ms ago

      for (const [dataset, currentValue] of zipTwo(datasets, currentValues)) {
        dataset.data.push({
          time: currentTime,
          x: 0,
          y: currentValue
        })
        // normalize x values so that the current value is at 0 and the oldest value is at -10
        for (let i = 0; i < dataset.data.length; i++) {
          dataset.data[i].x =
            (normalizeNumber(dataset.data[i].time, currentTime - 10000, currentTime) - 1) * 10
        }
        // remove values older than 10 seconds
        while (dataset.data[0].x < -10) {
          dataset.data.shift()
        }
      }
      chart.update()
    }
    animate()
    return () => {
      cancelAnimationFrame(animationFrameRequest!)
    }
  })
</script>

<Section>
  <div slot="header" class="flex items-baseline">
    <div>
      {header}
      <span class="text-gray-500 text-sm font-normal">{subtitle}</span>
    </div>
    <div class="ml-auto flex text-sm gap-4 font-normal">
      {#each datasets as dataset}
        <div>
          {dataset.label}
          <span style="color: {dataset.borderColor}"> ● </span>
        </div>
      {/each}
    </div>
  </div>
  <canvas bind:this={canvas} />
</Section>
