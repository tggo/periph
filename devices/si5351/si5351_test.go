package si5351

import (
	"testing"
		"periph.io/x/periph/conn/i2c/i2ctest"

	)

func TestDev_PLLCalc_simple_caclulation(t *testing.T) {
	bus := i2ctest.Playback{
		Ops: []i2ctest.IO{
			// Chip ID detection.
			{Addr: 0x60, W: []byte{0x0}, R: []byte{0x11}},
			{Addr: 0x60, W: []byte{0x0}, R: []byte{0x11}},
			{Addr: 0x60, W: []byte{0x1}, R: []byte{0x11}},
			{Addr: 0x60, W: []byte{0x3, 0xFF}, R: nil},
			// reseting
			{Addr: 0x60, W: []byte{0x10, 0x80}, R: nil},
			{Addr: 0x60, W: []byte{0x11, 0x80}, R: nil},
			{Addr: 0x60, W: []byte{0x12, 0x80}, R: nil},
			{Addr: 0x60, W: []byte{0x13, 0x80}, R: nil},
			{Addr: 0x60, W: []byte{0x14, 0x80}, R: nil},
			{Addr: 0x60, W: []byte{0x15, 0x80}, R: nil},
			{Addr: 0x60, W: []byte{0x16, 0x80}, R: nil},
			{Addr: 0x60, W: []byte{0x17, 0x80}, R: nil},
			// Set the load capacitance for the XTAL
			{Addr: 0x60, W: []byte{0xb7, 0xc0}, R: nil},
		},
		DontPanic: false,
	}

	dev, err := New(&bus, 0x60, &DefaultOpts)
	if dev == nil || err != nil {
		t.Fatal("read failed")
	}

	// The I/O didn't occur.
	//bus.Count++
	if err := bus.Close(); err != nil {
		t.Fatal(err)
	}

	p1,p2,p3 := dev.PLLCalc(0,616000000)
	if p1 != 24 || p2 != 640000 || p3 != 1000000 {
		t.Fatal("wrong calculation PLL", p1, p2, p3)
	}

	targetFreq := int64(13557000)

	p1,p2,p3, pllFreq := dev.CalcMultiSynth(targetFreq, 0)

	//fmt.Println(p1,p2,p3)

	clacFreq := int64( float64(pllFreq) / (float64(p1)+float64(p2)/float64(p3)) )

	if clacFreq!=targetFreq {
		t.Fatal("target Freq != calucluated Freq ", targetFreq, clacFreq)
	}
	//
	////t.Fatal(1)
}