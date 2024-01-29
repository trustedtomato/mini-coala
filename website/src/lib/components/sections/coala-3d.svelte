<script lang="ts">
  import { onMount } from 'svelte'
  import Section from './section.svelte'
  import * as THREE from 'three'
  import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader'
  import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module'
  import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'
  import debug from 'debug'
  import { getRad, pitchOffset, rollOffset, yawOffset } from '$lib/utils/get-rad'

  const loader = new GLTFLoader()
  loader.setMeshoptDecoder(MeshoptDecoder)

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
    camera.position.z = 6

    // --- scene ---

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0xe2e7ed)

    // AmbientLight adds a constant amount of light to every object from all directions
    const ambientLight = new THREE.AmbientLight(0xffffff, 1.6)
    scene.add(ambientLight)

    const pointLight = new THREE.PointLight(0xffffff, 15)
    camera.add(pointLight)
    scene.add(camera)
    // --- model ---

    const group = new THREE.Group()

    loader.load(
      'simplified/COALA5.glb',
      function (object) {
        const obj = object.scene
        obj.scale.setScalar(0.01)
        const boundingBox = new THREE.Box3()
        boundingBox.setFromObject(obj)
        const center = new THREE.Vector3()
        boundingBox.getCenter(center)
        // Use the center of the bounding box to move the center of the object to (0, 0, 0)
        obj.position.set(-center.x, -center.y, -center.z)
        group.add(obj)
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

    // --- renderer ---

    const renderer = new THREE.WebGLRenderer({ antialias: true })
    // use gamma output
    renderer.outputColorSpace = THREE.SRGBColorSpace
    renderer.setPixelRatio(window.devicePixelRatio)
    renderer.setSize(container.clientWidth, container.clientHeight)
    container.appendChild(renderer.domElement)

    //--- controls ---

    /*
    if (import.meta.env.DEV) {
      const controls = new OrbitControls(camera, renderer.domElement)
    }
    */

    // --- resize ---

    function onResize() {
      camera.aspect = container.clientWidth / container.clientHeight
      camera.updateProjectionMatrix()

      renderer.setSize(container.clientWidth, container.clientHeight)
    }

    window.addEventListener('resize', onResize)

    // --- animate ---

    let animationFrameRequest: number
    function animate() {
      animationFrameRequest = requestAnimationFrame(animate)
      if (group) {
        // console.log(roll, pitch, yaw)
        group.rotation.set(getRad(roll), getRad(-pitch), getRad(yaw+180), 'ZYX')
        group.rotateOnWorldAxis(new THREE.Vector3(1, 0, 0), getRad(pitchOffset))
        // group.rotation.x = getRad(pitch) + pitchOffset
        // group.rotation.y = getRad(roll) + rollOffset
        // group.rotation.z = getRad(yaw) + yawOffset
      }
      renderer.render(scene, camera)
    }
    animate()

    return () => {
      cancelAnimationFrame(animationFrameRequest)
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
