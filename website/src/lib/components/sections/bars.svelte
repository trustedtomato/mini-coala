<script lang="ts">
  import { onMount } from 'svelte'
  import Section from './section.svelte'
  import { normalizeNumber } from '$lib/utils/normalize-number'

  export let header = 'Bars'
  export let minValue = 0
  export let maxValue = 1
  export let barValues: number[] = []

  // only update the barValues on requestAnimationFrame
  let _barValues: number[] = []
  onMount(() => {
    let animationFrameRequest: number
    function update() {
      _barValues = barValues
      animationFrameRequest = requestAnimationFrame(update)
    }
    update()
    return () => {
      cancelAnimationFrame(animationFrameRequest)
    }
  })
</script>

<Section>
  <span slot="header">
    {header}
  </span>

  {#each _barValues as barValue}
    <div class="h-5 relative bg-x-blue-gray my-2">
      <div
        class="h-full origin-left bg-x-blue"
        style="transform: scaleX({normalizeNumber(barValue, minValue, maxValue) * 100}%);"
      />
    </div>
  {/each}
</Section>
