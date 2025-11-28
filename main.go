package main

import (
	"encoding/binary"
	"fmt"
	"log"
	"math"
	"os"
	"os/signal"
	"net"
	"syscall"
	"time"

	"periph.io/x/conn/v3/i2c"
	"periph.io/x/conn/v3/i2c/i2creg"
	"periph.io/x/host/v3"

	"github.com/gordonklaus/portaudio"
)

const (
	SERVO_ON_PIN = 12
	SERVO_OFF_PIN = 18
)

func main() {
	// signal channel to listen for interrupt signal
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt, syscall.SIGTERM)

	// init devices
	conn := initServos()
	times := make(chan time.Time)
	lightSensor, lightBus := initLightSensor()
	mic := initMic(times)
	prevClaptime := time.Now()

	// close device resources
	defer closeDeviceResources(mic, lightBus, conn)

	running := true

	// main loop
	for running {
		select {
		case <-sigChan:
			fmt.Println("Shutting down...")
			running = false
		
		case clapTime := <-times:
			if clapTime.Sub(prevClaptime).Milliseconds() > 100 && clapTime.Sub(prevClaptime).Milliseconds() < 500 {
				prevClaptime = clapTime
				fmt.Println("Clap 2")

				readCmd := []byte{0xB4}
				readBuffer := make([]byte, 2)
				if err := lightSensor.Tx(readCmd, readBuffer); err != nil {
					log.Fatalf("Failed to read from light sensor: %v", err)
				}

				// the high and low readings together make up the read value
				// bitshift the high reading by 8 bits
				// then bitwise or the 2 to combine them into a single int
				lightVal := uint16(readBuffer[1])<<8 | uint16(readBuffer[0])
				fmt.Println(lightVal)
				if lightVal < 3 {
					fmt.Println("Light Off")
					setServo(conn, SERVO_ON_PIN, angleToPulse(0))
					time.Sleep(500 * time.Millisecond)
					setServo(conn, SERVO_ON_PIN, angleToPulse(90))
				} else {
					fmt.Println("Light On")
					setServo(conn, SERVO_OFF_PIN, angleToPulse(180))
					time.Sleep(500 * time.Millisecond)
					setServo(conn, SERVO_OFF_PIN, angleToPulse(90))
				}
				for len(times) > 0 {
					<-times
				}
			} else if clapTime.Sub(prevClaptime).Milliseconds() > 200 {
				prevClaptime = clapTime
				fmt.Println("Clap 1")
			}
		}
	}

}

// servo functions
func initServos() net.Conn {
	conn, err := net.Dial("tcp", "localhost:8888")
	if err != nil {
		log.Fatal("Failed to connect. Run: sudo pigpiod")
	}

	// reset servo positions
	setServo(conn, SERVO_ON_PIN, angleToPulse(90))
	setServo(conn, SERVO_OFF_PIN, angleToPulse(90))

	return conn
}

func setServo(conn net.Conn, pin uint32, pulsewidth uint32) error {
	buf := make([]byte, 16)
	
	// CMD_SERVO
	binary.LittleEndian.PutUint32(buf[0:4], 8) 
	binary.LittleEndian.PutUint32(buf[4:8], pin)
	binary.LittleEndian.PutUint32(buf[8:12], pulsewidth)
	
	conn.Write(buf)
	response := make([]byte, 16)
	conn.Read(response)
	
	return nil
}

func angleToPulse(angle int) uint32 {
	return uint32(500 + (angle * 2000 / 180))
}

// light sensor functions
func initLightSensor() (*i2c.Dev, i2c.BusCloser) {
	if _, err := host.Init(); err != nil {
		log.Fatal(err)
	}

	bus, err := i2creg.Open("")
	if err != nil {
		log.Fatal(err)
	}

	lightSensor := &i2c.Dev{Addr: 0x29, Bus: bus}

	// write to wake, enable and record visible light
	wakeUpCmd := []byte{0xA0, 0x03}
	if err := lightSensor.Tx(wakeUpCmd, nil); err != nil {
		log.Fatal(err)
	}

	return lightSensor, bus
}

// mic functions
func initMic(times chan time.Time) *portaudio.Stream {
	if err := portaudio.Initialize(); err != nil {
		log.Fatalf("Failed to initialize PortAudio: %v", err)
	}

	mic, streamErr := portaudio.OpenDefaultStream(1, 0, 48000.0, 256, func(in []float32, out []float32) {
		clapThreshold := 0.10
		for i := 1; i < len(in)-1; i++ {
			prev := math.Abs(float64(in[i-1]))
			cur := math.Abs(float64(in[i]))
			next := math.Abs(float64(in[i+1]))
			if max(cur-prev) > clapThreshold || max(cur-next) > clapThreshold {
				now := time.Now()
				times <- now
				break
			}
		}
	})

	if streamErr != nil {
		log.Fatalf("Failed to open stream: %v", streamErr)
	}

	if err := mic.Start(); err != nil {
		log.Fatalf("Failed to start stream: %v", err)
	}

	fmt.Println("Mic Listening...")
	return mic
}

// clean up functions
func closeDeviceResources(mic *portaudio.Stream, lightBus i2c.BusCloser, conn net.Conn) {
	fmt.Println("Cleaning up microphone...")
	mic.Stop()
	mic.Close()
	portaudio.Terminate()

	fmt.Println("Cleaning up light sensor...")
	lightBus.Close()

	fmt.Println("Cleaning up servos...")
	conn.Close()
}