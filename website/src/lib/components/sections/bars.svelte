<script lang="ts">
  import { onMount } from 'svelte'
  import Section from './section.svelte'
  import { normalizeNumber } from '$lib/utils/normalize-number'

  export let header = 'Bars'
  export let minValue = 0
  export let maxValue = 1
  export let barValues: number[] = []
</script>

<Section>
  <span slot="header">
    {header}
  </span>

  {#each barValues as barValue}
    <div class="h-5 relative bg-x-blue-gray my-2">
      <div
        class="h-full origin-left bg-x-blue"
        style="transform: scaleX({normalizeNumber(barValue, minValue, maxValue) * 100}%);"
      />
      <span class="absolute left-1 top-0 leading-none bg-x-blue-gray px-1 mt-1 bg-opacity-75">
        {barValue.toPrecision(4)}
      </span>
      <span
        class="absolute right-1 top-0 leading-none bg-x-blue-gray px-1 mt-1 bg-opacity-75 text-xs"
      >
        {minValue} to {maxValue}
      </span>
    </div>
  {/each}
</Section>
