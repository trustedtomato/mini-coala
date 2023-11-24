/** @type {import('tailwindcss').Config} */
export default {
  content: ['./src/**/*.{html,js,svelte,ts}'],
  theme: {
    screens: {
      desktop: '1024px',
      bigDesktop: '1440px',
      veryBigDesktop: '1600px'
    },
    extend: {
      container: {
        center: true,
        padding: '1rem'
      },
      colors: {
        'x-blue': '#004286',
        'x-blue-gray': '#E2E7ED',
        'x-blue-light': '#7784F7',
        'x-gray': '#F5F5F5',
        'x-red': '#C01633',
        'x-black': '#000000',
        'x-white': '#FEFEFE',
        'x-green': '#19C016'
      }
    }
  },
  plugins: []
}
