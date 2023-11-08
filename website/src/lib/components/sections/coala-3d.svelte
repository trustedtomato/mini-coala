<script lang="ts">
  import { onMount } from 'svelte'
  import Section from './section.svelte'
  import * as THREE from 'three'
  import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader'
  import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader'
  import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'
  import debug from 'debug'

  const log = debug('app:coala-3d')

  export let header = 'Coala 3D'
  /** The pitch of the COALA in radians. Updated on requestAnimationFrame. */
  export let pitch = 0
  /** The roll of the COALA in radians. Updated on requestAnimationFrame. */
  export let roll = 0
  /** The yaw of the COALA in radians. Updated on requestAnimationFrame. */
  export let yaw = 0

  let container: HTMLElement

  onMount(() => {
    // --- camera ---

    const camera = new THREE.PerspectiveCamera(
      // vertical field of view (angle in degrees)
      45,
      // field of view aspect ratio (width / height)
      container.clientWidth / container.clientHeight,
      // near plane (how close to the camera objects can be rendered)
      0.1,
      // far plane (how far from the camera objects can be rendered)
      20
    )

    // zoom out to see the whole model
    camera.position.z = 6.5

    // --- scene ---

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0xe2e7ed)

    // AmbientLight adds a constant amount of light to every object from all directions
    const ambientLight = new THREE.AmbientLight(0xffffff)
    scene.add(ambientLight)

    const pointLight = new THREE.PointLight(0xffffff, 15)
    camera.add(pointLight)
    scene.add(camera)
    // --- model ---

    const group = new THREE.Group()

    new MTLLoader().load('COALA.mtl', function (materials) {
      materials.preload()

      new OBJLoader().setMaterials(materials).load(
        'COALA.obj',
        function (object) {
          // Scale the model down and rotate to make it upright
          object.scale.setScalar(0.01)
          object.rotateX(Math.PI / 2)
          const boundingBox = new THREE.Box3()
          boundingBox.setFromObject(object)
          const center = new THREE.Vector3()
          boundingBox.getCenter(center)
          // Use the center of the bounding box to move the center of the object to (0, 0, 0)
          object.position.set(-center.x, -center.y, -center.z)
          group.add(object)
          scene.add(group)
        },
        // on progress
        (xhr) => {
          if (xhr.lengthComputable) {
            const percentComplete = (xhr.loaded / xhr.total) * 100
            log(percentComplete.toFixed(2) + '% downloaded')
          }
        }
      )
    })

    // --- renderer ---

    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setPixelRatio(window.devicePixelRatio)
    renderer.setSize(container.clientWidth, container.clientHeight)
    container.appendChild(renderer.domElement)

    //--- controls ---

    if (import.meta.env.DEV) {
      const controls = new OrbitControls(camera, renderer.domElement)
    }

    // --- resize ---

    function onResize() {
      camera.aspect = container.clientWidth / container.clientHeight
      camera.updateProjectionMatrix()

      renderer.setSize(container.clientWidth, container.clientHeight)
    }

    window.addEventListener('resize', onResize)

    // --- animate ---

    function animate() {
      requestAnimationFrame(animate)
      if (group) {
        group.rotation.y = yaw
        group.rotation.x = pitch
        group.rotation.z = roll
      }
      renderer.render(scene, camera)
    }
    animate()

    return () => {
      window.removeEventListener('resize', onResize)
    }
  })
</script>

<Section>
  <span slot="header">
    {header}
  </span>
  <div bind:this={container} class="w-full aspect-square bg-x-blue-gray" />
</Section>
