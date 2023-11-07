<script lang="ts">
  import { Chart, registerables, type ChartDataset } from 'chart.js'

  export let header = 'Chart'
  export let subtitle = '(m / s)'
  export let datasets: ChartDataset<'scatter'>[] = [
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
      showLine: true,
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

  const datasetDefaults: Partial<ChartDataset<'scatter'>> = {
    showLine: true,
    borderCapStyle: 'round',
    pointRadius: 0,
  }

  $: chartjsDatasets = datasets.map((dataset) => ({
    ...datasetDefaults,
    ...dataset
  }))

  Chart.register(...registerables)

  let canvas: HTMLCanvasElement | undefined
  $: ctx = canvas?.getContext('2d')
  $: if (ctx) {
    new Chart(ctx, {
      type: 'scatter',
      data: {
        datasets: chartjsDatasets
      },
      options: {
        responsive: true,
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
  }

</script>

<h2 class="mt-8 mb-2">
  <div class="flex items-baseline">
    <div>
      {header}
      <span class="text-gray-500 text-sm">{subtitle}</span>
    </div>
    <div class="ml-auto flex text-sm gap-4">
      {#each datasets as dataset}
      <div>
        {dataset.label}
        <span style="color: {dataset.borderColor}"> ● </span>
      </div>
      {/each}
    </div>
  </div>
</h2>

<canvas bind:this={canvas}></canvas>