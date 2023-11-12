import rosnodejs from 'rosnodejs'
const std_msgs = rosnodejs.require('std_msgs')
const sensor_msgs = rosnodejs.require('sensor_msgs')
import { WebSocketServer } from 'ws'

await rosnodejs.initNode('/my_node', { rosMasterUri: 'http://raspberrypi:11311' })
console.log('ROS node initialized')
const nh = rosnodejs.nh
const publisher = nh.advertise('/joystick', std_msgs.msg.Float32MultiArray)
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
    publisher.publish(message)
  })

  webSocket = socket
  socket.send('test from server')
})


nh.subscribe('/motor_cmd', std_msgs.msg.Float32MultiArray, (msg) => {
  rosnodejs.log.info(`Recieved ${msg.data}`)
  webSocket?.send(JSON.stringify({ type: 'motor', data: msg.data }))
})

nh.subscribe('/pressure_sensor', std_msgs.msg.Float32MultiArray, (msg) => {
  rosnodejs.log.info(`Recieved ${msg.data}`)
  webSocket?.send(JSON.stringify({ type: 'pressure', data: msg.data }))
})

nh.subscribe('/imu/data', sensor_msgs.msg.Imu, (msg) => {
  rosnodejs.log.info(`Recieved ${msg.data}`)
  webSocket?.send(JSON.stringify({ type: 'imu', data: msg.orientation }))
})

