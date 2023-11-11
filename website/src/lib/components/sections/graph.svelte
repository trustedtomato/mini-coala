<script lang="ts">
  import { Chart, registerables, type ChartDataset } from 'chart.js'
  import Section from './section.svelte'
  import { onMount } from 'svelte'

  export let header = 'Chart'
  export let subtitle = ''
  export let yAxisLabel = 'Heave (m)'
  export let ySuggestedMin = 0
  export let ySuggestedMax = 1

  export let datasets: ChartDataset<'line'>[] = [
    {
      label: 'Dataset 1',
      data: [
        {
          x: 0,
          y: 0.5
        },
        {
          x: 0.5,
          y: 0.75
        },
        {
          x: 1,
          y: 0.5
        }
      ],
      borderColor: '#C01633'
    },
    {
      label: 'Dataset 2',
      borderColor: '#7784F7',
      data: [
        {
          x: 0,
          y: 0.5
        },
        {
          x: 0.5,
          y: 0.5
        },
        {
          x: 1,
          y: 0.25
        }
      ]
    }
  ]

  // added some defaults as per:
  // https://www.chartjs.org/docs/latest/general/performance.html
  const datasetDefaults: Partial<ChartDataset<'line'>> = {
    borderCapStyle: 'round',
    showLine: true,
    pointRadius: 0,
    spanGaps: true
  }

  $: chartjsDatasets = datasets.map((dataset) => ({
    ...datasetDefaults,
    ...dataset
  }))

  Chart.register(...registerables)

  let canvas: HTMLCanvasElement | undefined

  onMount(() => {
    const ctx = canvas!.getContext('2d')!
    const chart = new Chart(ctx, {
      type: 'scatter',
      data: {
        datasets: chartjsDatasets
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
            min: ySuggestedMin,
            max: ySuggestedMax
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
      chart.update()
    }
    animate()
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
