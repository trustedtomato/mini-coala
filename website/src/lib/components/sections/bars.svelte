<script lang="ts">
  import Section from './section.svelte'

  export let header = 'Bars'
  export let barValues: number[] = []
</script>

<Section>
  <span slot="header">
    {header}
  </span>

  {#each barValues as barValue}
    <div class="h-5 relative bg-x-blue-gray my-2">
      <div class="h-full origin-left bg-x-blue" style="transform: scaleX({barValue * 100}%);" />
    </div>
  {/each}
</Section>
