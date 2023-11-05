import rosnodejs from 'rosnodejs'
const std_msgs = rosnodejs.require('std_msgs')
import { WebSocketServer } from 'ws'

await rosnodejs.initNode('/my_node', { rosMasterUri: 'http://raspberrypi:11311' })
console.log('ROS node initialized')
const nh = rosnodejs.nh
const publisher = nh.advertise('/Joystick', std_msgs.msg.Float32MultiArray)
let webSocket
const webSocketServer = new WebSocketServer({
  port: 3000
})

webSocketServer.on('connection', (socket, request) => {
  socket.on('message', (data, isBinary) => {
    console.log(`Recieved ${data}`)
  })

  webSocket = socket
  socket.send('test from server')
})

webSocketServer.on('message', (data, isBinary) => {
  rosnodejs.log.info(`Recieved ${data}`)
  const message = new std_msgs.Float32MultiArray({
    data: data
  })
  publisher.publish(message)
})

const subscriber = nh.subscribe('/motor_cmd', std_msgs.msg.Float32MultiArray, (msg) => {
  rosnodejs.log.info(`Recieved ${msg.data}`)
  webSocket?.send(JSON.stringify(msg.data))
})
