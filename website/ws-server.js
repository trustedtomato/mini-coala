import rosnodejs from 'rosnodejs'
const std_msgs = rosnodejs.require('std_msgs')
const sensor_msgs = rosnodejs.require('sensor_msgs')
const rosbno055_msgs = rosnodejs.require('ros_bno055')
const geometry_msgs = rosnodejs.require('geometry_msgs')
import { WebSocketServer } from 'ws'

await rosnodejs.initNode('/my_node', { rosMasterUri: 'http://raspberrypi:11311' })
console.log('ROS node initialized')
const nh = rosnodejs.nh
const joystickDataPublisher = nh.advertise('/joystick_data', std_msgs.msg.Float32MultiArray)
let webSocket
const webSocketServer = new WebSocketServer({
  port: 3000
})

webSocketServer.on('connection', (socket, request) => {
  socket.on('message', (data, isBinary) => {
    console.log(`Recieved ${data}`)
    const message = new std_msgs.msg.Float32MultiArray({
      data: data
    })
    joystickDataPublisher.publish(message)
  })

  webSocket?.close()
  webSocket = socket
  socket.send('test from server')
})

nh.subscribe('/motor_cmd', std_msgs.msg.Float32MultiArray, (msg) => {
  rosnodejs.log.info(`Recieved ${msg.data}`)
  webSocket?.send(JSON.stringify({ type: 'motor', data: msg.data }))
})

nh.subscribe('/heave_data', std_msgs.msg.Float32, (msg) => {
  rosnodejs.log.info(`Recieved ${msg.data}`)
  webSocket?.send(JSON.stringify({ type: 'heave', data: msg.data }))
})

nh.subscribe('/imu/orientation_euler/calibrated', rosbno055_msgs.msg.OrientationEuler, (msg) => {
  // inverse the roll and pitch

  rosnodejs.log.info(`Recieved ${(msg.heading, msg.pitch, msg.roll)}`)
  webSocket?.send(
    JSON.stringify({
      type: 'imu_euler',
      data: {
        yaw: msg.heading,
        pitch: msg.pitch,
        roll: msg.roll
      }
    })
  )
})

nh.subscribe('/imu/orientation_quat/calibrated', geometry_msgs.msg.Quaternion, (msg) => {
  rosnodejs.log.info(`Recieved ${msg}`)
  webSocket?.send(JSON.stringify({ type: 'imu_quaternion', data: msg }))
})