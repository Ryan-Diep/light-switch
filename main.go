package main

import (
	"fmt"
	"log"
	"math"
	"time"

	"periph.io/x/conn/v3/gpio"
	"periph.io/x/conn/v3/gpio/gpioreg"
	"periph.io/x/conn/v3/i2c"
	"periph.io/x/conn/v3/i2c/i2creg"
	"periph.io/x/conn/v3/physic"
	"periph.io/x/host/v3"

	"github.com/gordonklaus/portaudio"
)

func main() {
	servoOnName := "GPIO23"
	servoOffName := "GPIO25"
	servoOn, servoOff := initServos(servoOnName, servoOffName)

	lightSensor := initLightSensor()

	times := make(chan time.Time)
	initMic(times)

	prevClaptime := time.Now()
	for {
		clapTime := <-times

		if clapTime.Sub(prevClaptime).Milliseconds() > 30 && clapTime.Sub(prevClaptime).Milliseconds() < 500 {
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
			if lightVal < 2 {
				fmt.Println("Light Off")
				fmt.Println("Activate Servo On")
				continue
				activateServo(servoOn)
			} else if false {
				fmt.Println("Light On")
				fmt.Println("Activate Servo Off")
				continue
				activateServo(servoOff)
			}

		} else if clapTime.Sub(prevClaptime).Milliseconds() > 200 {
			prevClaptime = clapTime
			fmt.Println("Clap 1")
		}
	}

}

func initServos(servoOnName string, servoOffName string) (gpio.PinIO, gpio.PinIO) {
	if _, err := host.Init(); err != nil {
		log.Fatal(err)
	}

	servoOn := gpioreg.ByName(servoOnName)
	if servoOn == nil {
		log.Fatalf("Failed to find %s", servoOn)
	}
	fmt.Printf("%s is %s\n", servoOn, servoOn.Read())

	servoOff := gpioreg.ByName(servoOffName)
	if servoOff == nil {
		log.Fatalf("Failed to find %s", servoOff)
	}
	fmt.Printf("%s is %s\n", servoOff, servoOff.Read())

	// init servo to neutral position
	// if err := servoOn.PWM(gpio.DutyMax*75/1000, 50*physic.Hertz); err != nil {
	// 	log.Fatal(err)
	// }
	//
	// time.Sleep(1 * time.Second)
	// if err := servoOff.PWM(gpio.DutyMax*75/1000, 50*physic.Hertz); err != nil {
	// 	log.Fatal(err)
	// }

	return servoOn, servoOff
}

func activateServo(servo gpio.PinIO) {
	// move servo to press light switch
	if err := servo.PWM(gpio.DutyMax*5/100, 50*physic.Hertz); err != nil {
		log.Fatalf("Failed to activate servo: %v", err)
	}

	time.Sleep(1 * time.Second)
	// bring servo back to neutral position
	if err := servo.PWM(gpio.DutyMax*75/1000, 50*physic.Hertz); err != nil {
		log.Fatalf("Failed to reset servo: %v", err)
	}
}

func initLightSensor() i2c.Dev {
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

	return *lightSensor
}

func initMic(times chan time.Time) {
	if err := portaudio.Initialize(); err != nil {
		log.Fatalf("Failed to initialize PortAudio: %v", err)
	}

	mic, streamErr := portaudio.OpenDefaultStream(1, 0, 48000.0, 256, func(in []float32, out []float32) {
		for i := 1; i < len(in)-1; i++ {
			clapThreshold := 0.10

			prev := math.Abs(float64(in[i-1]))
			cur := math.Abs(float64(in[i]))
			next := math.Abs(float64(in[i+1]))
			if max(cur-prev) > clapThreshold || max(cur-next) > clapThreshold {
				times <- time.Now()
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
}
