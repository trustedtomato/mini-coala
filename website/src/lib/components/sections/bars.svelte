<script lang="ts">
  export let header = 'Bars'
  export let barValues: number[] = []
</script>

<h2 class="mt-8 mb-2">
  {header}
</h2>

{#each barValues as barValue}
  <div class="h-5 relative bg-x-blue-gray my-2">
    <div class="h-full origin-left bg-x-blue" style="transform: scaleX({barValue * 100}%);" />
  </div>
{/each}
