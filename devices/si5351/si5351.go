package si5351

import (
	"errors"
	"periph.io/x/periph/conn"
	"periph.io/x/periph/conn/i2c"
	"sync"
	"time"

	"fmt"
	"strings"
)

type Filter uint8

// source https://github.com/etherkit/Si5351Arduino/blob/master/src/si5351.h
// https://github.com/etherkit/Si5351Arduino/blob/master/src/si5351.cpp
// http://dspview.com/viewtopic.php?f=22&t=162

// Possible filtering values.
const (
	deviceStatus                         = 0
	interruptStatusSticky                = 1
	interruptStatusMask                  = 2
	outputEnableControl                  = 3
	oebPinEnableControl                  = 9
	pllInputSource                       = 15
	clk0Control                          = 16
	clk1Control                          = 17
	clk2Control                          = 18
	clk3Control                          = 19
	clk4Control                          = 20
	clk5Control                          = 21
	clk6Control                          = 22
	clk7Control                          = 23
	REGISTER_24_CLK3_0_DISABLE_STATE     = 24
	REGISTER_25_CLK7_4_DISABLE_STATE     = 25
	REGISTER_42_MULTISYNTH0_PARAMETERS_1 = 42
	REGISTER_43_MULTISYNTH0_PARAMETERS_2 = 43
	REGISTER_44_MULTISYNTH0_PARAMETERS_3 = 44
	REGISTER_45_MULTISYNTH0_PARAMETERS_4 = 45
	REGISTER_46_MULTISYNTH0_PARAMETERS_5 = 46
	REGISTER_47_MULTISYNTH0_PARAMETERS_6 = 47
	REGISTER_48_MULTISYNTH0_PARAMETERS_7 = 48
	REGISTER_49_MULTISYNTH0_PARAMETERS_8 = 49
	REGISTER_50_MULTISYNTH1_PARAMETERS_1 = 50
	REGISTER_51_MULTISYNTH1_PARAMETERS_2 = 51
	REGISTER_52_MULTISYNTH1_PARAMETERS_3           = 52
	REGISTER_53_MULTISYNTH1_PARAMETERS_4           = 53
	REGISTER_54_MULTISYNTH1_PARAMETERS_5           = 54
	REGISTER_55_MULTISYNTH1_PARAMETERS_6           = 55
	REGISTER_56_MULTISYNTH1_PARAMETERS_7           = 56
	REGISTER_57_MULTISYNTH1_PARAMETERS_8           = 57
	REGISTER_58_MULTISYNTH2_PARAMETERS_1           = 58
	REGISTER_59_MULTISYNTH2_PARAMETERS_2           = 59
	REGISTER_60_MULTISYNTH2_PARAMETERS_3           = 60
	REGISTER_61_MULTISYNTH2_PARAMETERS_4           = 61
	REGISTER_62_MULTISYNTH2_PARAMETERS_5           = 62
	REGISTER_63_MULTISYNTH2_PARAMETERS_6           = 63
	REGISTER_64_MULTISYNTH2_PARAMETERS_7           = 64
	REGISTER_65_MULTISYNTH2_PARAMETERS_8           = 65
	REGISTER_66_MULTISYNTH3_PARAMETERS_1           = 66
	REGISTER_67_MULTISYNTH3_PARAMETERS_2           = 67
	REGISTER_68_MULTISYNTH3_PARAMETERS_3           = 68
	REGISTER_69_MULTISYNTH3_PARAMETERS_4           = 69
	REGISTER_70_MULTISYNTH3_PARAMETERS_5           = 70
	REGISTER_71_MULTISYNTH3_PARAMETERS_6           = 71
	REGISTER_72_MULTISYNTH3_PARAMETERS_7           = 72
	REGISTER_73_MULTISYNTH3_PARAMETERS_8           = 73
	REGISTER_74_MULTISYNTH4_PARAMETERS_1           = 74
	REGISTER_75_MULTISYNTH4_PARAMETERS_2           = 75
	REGISTER_76_MULTISYNTH4_PARAMETERS_3           = 76
	REGISTER_77_MULTISYNTH4_PARAMETERS_4           = 77
	REGISTER_78_MULTISYNTH4_PARAMETERS_5           = 78
	REGISTER_79_MULTISYNTH4_PARAMETERS_6           = 79
	REGISTER_80_MULTISYNTH4_PARAMETERS_7           = 80
	REGISTER_81_MULTISYNTH4_PARAMETERS_8           = 81
	REGISTER_82_MULTISYNTH5_PARAMETERS_1           = 82
	REGISTER_83_MULTISYNTH5_PARAMETERS_2           = 83
	REGISTER_84_MULTISYNTH5_PARAMETERS_3           = 84
	REGISTER_85_MULTISYNTH5_PARAMETERS_4           = 85
	REGISTER_86_MULTISYNTH5_PARAMETERS_5           = 86
	REGISTER_87_MULTISYNTH5_PARAMETERS_6           = 87
	REGISTER_88_MULTISYNTH5_PARAMETERS_7           = 88
	REGISTER_89_MULTISYNTH5_PARAMETERS_8           = 89
	REGISTER_90_MULTISYNTH6_PARAMETERS     = 90
	REGISTER_91_MULTISYNTH7_PARAMETERS     = 91
	REGISTER_092_CLOCK_6_7_OUTPUT_DIVIDER  = 92
	REGISTER_165_CLK0_INITIAL_PHASE_OFFSET = 165
	REGISTER_166_CLK1_INITIAL_PHASE_OFFSET = 166
	REGISTER_167_CLK2_INITIAL_PHASE_OFFSET = 167
	REGISTER_168_CLK3_INITIAL_PHASE_OFFSET = 168
	REGISTER_169_CLK4_INITIAL_PHASE_OFFSET = 169
	REGISTER_170_CLK5_INITIAL_PHASE_OFFSET = 170
	REGISTER_177_PLL_RESET                 = 177
	crystalInternalLoadCapacitance         = 183

	pllReset  = 177
	pllResetB = 1 << 7
	pllResetA = 1 << 5

	crystalLoad     = 183
	crystalLoadMask = (3 << 6)
	crystalLoad0PF  = (0 << 6)
	crystalLoad6PF  = (1 << 6)
	crystalLoad8PF  = (2 << 6)
	crystalLoad10PF = (3 << 6)

	crystalFreq25MHZ = 25000000
	crystalFreq27MHZ = 27000000

	pllFixed   = 80000000000
	freqMult   = 1
	defaultClk = 1000000000

	pllVcoMin = 600000000
	pllVcoMax = 900000000

	parametersLength = 8
	pllAParameters   = 26
	pllBParameters   = 34
	clk0Parameters   = 42
	clk1Parameters   = 50
	clk2Parameters   = 58
	clk3Parameters   = 66
	clk4Parameters   = 74
	clk5Parameters   = 82
	clk6Parameters   = 90

	fanoutEnable     = 187
	clkInEnable      = (1 << 7)
	xtalEnable       = (1 << 6)
	multiSynthEnable = (1 << 4)

	multiSynthMinFreq    = 500000
	multiSynthMaxFreq    = 225000000
	multiSynthShareMax   = 100000000
	multiSynthShareMin   = 1024000
	multisynthDivby4Freq = 150000000
	multisynth67MaxFreq  = multisynthDivby4Freq
	clkoutMinFreq        = 4000
	clkoutMaxFreq        = multiSynthMaxFreq
	clkout67MsMin        = pllVcoMin / multisynth67AMax

	multisynthAMin   = 6
	multisynthAMax   = 1800
	multisynth67AMax = 254
	multisynthBMax   = (multisynthCMax - 1)
	multisynthCMax   = 1048575
	multisynthP1Max  = ((1 << 18) - 1)
	multisynthP2Max  = ((1 << 20) - 1)
	multisynthP3Max  = ((1 << 20) - 1)
	vcxoPullMin      = 30
	vcxoPullMax      = 240
	vcxoMargin       = 103

	rfracDenom = 1000000

	pllAMin = 24
	pllAMax = 36
	pllBMax = (pllCMax - 1)
	pllCMax = 1048575

	PLLA = 0
	PLLB = 1

	outputClkDiv1   = 0
	outputClkDiv2   = 1
	outputClkDiv4   = 2
	outputClkDiv8   = 3
	outputClkDiv16  = 4
	outputClkDiv32  = 5
	outputClkDiv64  = 6
	outputClkDiv128 = 7
	outputClkDivby4 = 3 << 2

	outputClkDivShift = 4

	clkIntegerMode = 1 << 6
)

// DefaultOpts is the recommended default options.
var DefaultOpts = Opts{
	crystalFreq:   crystalFreq25MHZ,
	crystalLoad:   crystalLoad10PF,
	crystalPPM:    30,
	pllaFreq:      0,
	pllbFreq:      0,
	refCorrection: 0,
	xoFreq:        0,
}

type Opts struct {
	crystalFreq   int64
	crystalLoad   byte
	crystalPPM    int32
	pllaFreq      int32
	pllbFreq      int32
	refCorrection int32
	xoFreq        int32
}

// Example: sysInit: 0  lolA: 0  lolB: 0  los: 1  revID: 1
type status struct {
	sysInit int32
	lolB    int32
	lolA    int32
	los     int32
	revID   int32

	sysInitStky int32
	lolBStky    int32
	lolAStky    int32
	losStky     int32
}

type Dev struct {
	d           conn.Conn
	crystalFreq int64
	crystalLoad byte
	crystalPPM  int32

	refCorrection int32

	//pllAFreq float64
	//pllAFreq float64

	pllAFreq int32
	pllBFreq int32
	xoFreq   int32

	//opts      Opts
	measDelay time.Duration
	name      string
	os        uint8

	Status status

	clkFreq      []int32
	clkFirstSet  []bool

	mu   sync.Mutex
	stop chan struct{}
	wg   sync.WaitGroup
}

func New(b i2c.Bus, addr uint16, opts *Opts) (*Dev, error) {

	switch addr {
	case 0x60, 0x62:
	default:
		return nil, errors.New("si5351: given address not supported by device")
	}

	d := &Dev{d: &i2c.Dev{Bus: b, Addr: addr}}

	// Check SYS_INIT flag to be clear, indicating that device is ready
	regValue, _ := d.Read(deviceStatus)

	if regValue[0]>>7 == 1 {
		return nil, errors.New("si5351: not ready")
	}

	//  read chip status
	if err := d.UpdateSysStatus(); err != nil {
		return nil, err
	}

	if err := d.UpdateIntStatus(); err != nil {
		return nil, err
	}

	// create Device with giving options
	if err := d.makeDev(opts); err != nil {
		return nil, err
	}

	return d, nil
}

func (d *Dev) makeDev(opts *Opts) error {
	//d.opts = *opts
	d.crystalFreq = opts.crystalFreq
	d.refCorrection = opts.refCorrection
	d.crystalLoad = opts.crystalLoad

	d.clkFirstSet = []bool{false, false, false, false, false, false}
	d.clkFreq = []int32{0, 0, 0, 0, 0, 0}

	// Disable all outputs setting CLKx_DIS high
	d.WriteCommand(outputEnableControl, 0xFF)

	// Power down all output drivers
	d.WriteCommand(clk0Control, 0x80)
	d.WriteCommand(clk1Control, 0x80)
	d.WriteCommand(clk2Control, 0x80)
	d.WriteCommand(clk3Control, 0x80)
	d.WriteCommand(clk4Control, 0x80)
	d.WriteCommand(clk5Control, 0x80)
	d.WriteCommand(clk6Control, 0x80)
	d.WriteCommand(clk7Control, 0x80)

	// Set the load capacitance for the XTAL
	d.WriteCommand(crystalInternalLoadCapacitance, d.crystalLoad)

	//// Set up the XO reference frequency
	//// NOTE: don't have chip for testing
	//if d.opts.xoFreq != 0 {
	//	d.setRefFreq(d.opts.xoFreq, PLL_INPUT_XO)
	//} else {
	//	d.setRefFreq(d.opts.crystalFreq, PLL_INPUT_XO)
	//}

	return nil
}

func (d *Dev) Reset() {
	// Disable all outputs setting CLKx_DIS high
	d.WriteCommand(outputEnableControl, 0xFF)

	// Power down all output drivers
	d.WriteCommand(clk0Control, 0x80)
	d.WriteCommand(clk1Control, 0x80)
	d.WriteCommand(clk2Control, 0x80)
	d.WriteCommand(clk3Control, 0x80)
	d.WriteCommand(clk4Control, 0x80)
	d.WriteCommand(clk5Control, 0x80)
	d.WriteCommand(clk6Control, 0x80)
	d.WriteCommand(clk7Control, 0x80)
}

func (d *Dev) PLLReset(targetPLL int32) {
	if targetPLL == PLLA {
		d.WriteCommand(pllReset, pllResetA)
	} else if targetPLL == PLLB {
		d.WriteCommand(pllReset, pllResetB)
	}
}

// Sets the multiplier for the specified PLL
// PLL Configuration
//  	 fVCO is the PLL output, and must be between 600..900MHz, where:
//		 fVCO = fXTAL * (a+(b/c))
//		 		fXTAL = the crystal input frequency
//		 		a     = an integer between 15 and 90
//				b     = the fractional numerator (0..1,048,575)
//				c     = the fractional denominator (1..1,048,575)
//
// Try to use integers whenever possible to avoid clock jitter
// (only use the a part, setting b to '0' and c to '1').
// See: http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf
//
// Feedback Multisynth Divider Equation
// where: a = mult, b = num and c = denom
//  P1[17:0] = 128 * mult + floor(128*(num/denom)) - 512
//  P2[19:0] = 128 * num - denom * floor(128*(num/denom))
//  P3[19:0] = denom
//
//
// pll   The PLL to configure, which must be one of the following:
//        - SI5351_PLL_A
//        - SI5351_PLL_B
// mult  The PLL integer multiplier (must be between 15 and 90)
// num   The 20-bit numerator for fractional output (0..1,048,575).
//		 Set this to '0' for integer output.
// denom The 20-bit denominator for fractional output (1..1,048,575).
//       Set this to '1' or higher to avoid divider by zero errors.
//
func (d *Dev) SetupPLL(pll uint, mult, num, denom int64) {

	// Set the main PLL config registers
	p1 := 128*mult + int64(128.0*num/denom) - 512
	p2 := 128*num - denom*int64(128.0*num/denom)
	p3 := denom
	var baseaddr byte

	// Get the appropriate starting point for the PLL registers
	if pll == PLLA {
		baseaddr = 26
	} else if pll == PLLB {
		baseaddr = 34
	}

	buff := []byte{
		baseaddr,
		byte((p3 & 0x0000FF00) >> 8),
		byte(p3 & 0x000000FF),
		byte((p1 & 0x00030000) >> 16),
		byte((p1 & 0x0000FF00) >> 8),
		byte(p1 & 0x000000FF),
		byte(byte((p3&0x000F0000)>>12)) | (byte((p2 & 0x000F0000) >> 16)),
		byte((p2 & 0x0000FF00) >> 8),
		byte(p2 & 0x000000FF),
	}

	if err := d.d.Tx(buff, nil); err != nil {
		fmt.Println(err)
	}

	// Reset both PLLs
	d.WriteCommand(pllReset, (1<<7)|(1<<5))

	// Store the frequency settings for use with the MultiSynth helper
	fvco := int32( float64(d.crystalFreq * (mult+num)) / float64(denom) )
	if pll == PLLA {
		d.pllAFreq = fvco
	} else if pll == PLLB {
		d.pllBFreq = fvco
	}
}

// Configures the Multisynth divider, which determines the output clock frequency based on the specified PLL input.
// num=0, denom=1
func (d *Dev) SetupMultiSynthNew(clk int8, pll, div, num, denom int64, intMode bool, rDiv int32, divBy4 int32) {
	// Set the main PLL config registers
	p1 := 128*div + int64(128.0*float64(num)/float64(denom)) - 512
	p2 := 128*num - denom*int64(128.0*float64(num)/float64(denom))
	p3 := denom
	var baseaddr byte

	// Get the appropriate starting point for the PLL registers
	if clk == 0 {
		baseaddr = REGISTER_42_MULTISYNTH0_PARAMETERS_1
	} else if clk == 1 {
		baseaddr = REGISTER_50_MULTISYNTH1_PARAMETERS_1
	} else if clk == 2 {
		baseaddr = REGISTER_58_MULTISYNTH2_PARAMETERS_1
	}

	buff := []byte{
		baseaddr,
		byte((p3 >> 8) & 0xFF),
		byte(p3 & 0xFF),
		byte((p1 >> 16) & 0x03),
		byte((p1 >> 8) & 0xFF),
		byte(p1 & 0xFF),
		byte(byte((p3>>12)&0xF0)) | (byte((p2 >> 16) & 0x0F)),
		byte((p2 >> 8) & 0xFF),
		byte(p2 & 0xFF),
	}

	if err := d.d.Tx(buff, nil); err != nil {
		fmt.Println(err)
	}

	d.SetupIntMode(int32(clk), intMode)
	d.MsDiv(int32(clk), rDiv, divBy4)

}

// Configures the Multisynth divider, which determines the output clock frequency based on the specified PLL input.
// num=0, denom=1
func (d *Dev) SetupMultiSynth(clk int8, pll, div, num, denom int64) {
	// Set the main PLL config registers
	p1 := 128*div + int64(128.0*float64(num)/float64(denom)) - 512
	p2 := 128*num - denom*int64(128.0*float64(num)/float64(denom))
	p3 := denom
	var baseaddr byte

	// Get the appropriate starting point for the PLL registers
	if clk == 0 {
		baseaddr = REGISTER_42_MULTISYNTH0_PARAMETERS_1
	} else if clk == 1 {
		baseaddr = REGISTER_50_MULTISYNTH1_PARAMETERS_1
	} else if clk == 2 {
		baseaddr = REGISTER_58_MULTISYNTH2_PARAMETERS_1
	}

	buff := []byte{
		baseaddr,
		byte((p3 >> 8) & 0xFF),
		byte(p3 & 0xFF),
		byte((p1 >> 16) & 0x03),
		byte((p1 >> 8) & 0xFF),
		byte(p1 & 0xFF),
		byte(byte((p3>>12)&0xF0)) | (byte((p2 >> 16) & 0x0F)),
		byte((p2 >> 8) & 0xFF),
		byte(p2 & 0xFF),
	}

	if err := d.d.Tx(buff, nil); err != nil {
		fmt.Println(err)
	}

	//d.SetupIntMode(int32(clk), true)

	// Configure the clk control and enable the output
	clkControlReg := byte(0x0F) // 8mA drive strength, MS0 as CLK0 source, Clock not inverted, powered up
	if pll == PLLB {
		clkControlReg |= 1 << 5
	}
	if num == 0 {
		clkControlReg |= 1 << 6
	}
	if clk == 0 {
		d.WriteCommand(clk0Control, clkControlReg) // 0x10
	} else if clk == 1 {
		d.WriteCommand(clk1Control, clkControlReg) // 0x10
	} else if clk == 2 {
		d.WriteCommand(clk2Control, clkControlReg) // 0x10

	}
}

func (d *Dev) SetupRdiv(output int32, div int32) {
	var Rreg byte

	if output == 0 {
		Rreg = REGISTER_44_MULTISYNTH0_PARAMETERS_3
	}
	if output == 1 {
		Rreg = REGISTER_52_MULTISYNTH1_PARAMETERS_3
	}
	if output == 2 {
		Rreg = REGISTER_60_MULTISYNTH2_PARAMETERS_3
	}

	d.WriteCommand(Rreg, (byte(div%0x07))<<4)
}

func (d *Dev) SetupIntMode(clk int32, status bool) {
	buff, _ := d.Read(byte(clk0Control + clk))
	regValue := buff[0]

	if status {
		regValue |= byte(clkIntegerMode)
	} else {
		regValue &= ^(byte(clkIntegerMode))
	}
	d.WriteCommand(byte(clk0Control+clk), regValue)
}

func (d *Dev) MsDiv(clk int32, rDiv, divBy4 int32) {
	address := 0
	if clk == 0 {
		address = clk0Parameters + 2
	}
	if clk == 1 {
		address = clk1Parameters + 2
	}
	if clk == 2 {
		address = clk2Parameters + 2
	}
	if clk == 3 {
		address = clk3Parameters + 2
	}
	if clk == 4 {
		address = clk3Parameters + 2
	}
	if clk == 5 {
		address = clk4Parameters + 2
	}
	if clk == 6 {
		address = clk5Parameters + 2
	}

	buff, _ := d.Read(byte(address))
	regValue := buff[0]

	if clk <= 5 {
		// Clear the relevant bits
		regValue &= ^(byte(0x7c))
		if divBy4 == 0 {
			regValue &= ^byte(outputClkDivby4)
		} else {
			regValue |= outputClkDivby4
		}
		regValue |= byte(rDiv) << byte(outputClkDivShift)
	} else if clk == 6 {
		// Clear the relevant bits
		regValue &= ^(byte(0x07))
		regValue |= byte(rDiv)

	} else if clk == 7 {
		// Clear the relevant bits
		regValue &= ^(byte(0x70))
		regValue |= byte(rDiv) << byte(outputClkDivShift)
	}

	d.WriteCommand(byte(address), regValue)

}

// Sets the clock frequency of the specified CLK output.
// Frequency range of 8 kHz to 150 MHz
//  freq - Output frequency in Hz
//  clk  - Clock output
func (d *Dev) SetupFreq(clk uint, freq int32) {
	intMode := false
	divBy4 := 0
	rDiv := 0

	// Check which Multisynth is being set
	if clk <= 5 {
		// MS0....MS5 logic
		// Lower bounds check
		if freq > 0 && freq < clkoutMinFreq*freqMult {
			freq = clkoutMinFreq * freqMult
		}

		// Upper bounds check
		if freq > multiSynthMaxFreq*freqMult {
			freq = multiSynthMaxFreq * freqMult
		}

		// If requested freq >100 MHz and no other outputs are already >100 MHz,
		// we need to recalculate PLLA and then recalculate all other CLK outputs
		// on same PLL
		if freq > (multiSynthShareMax * freqMult) {
			// Check other clocks on same PLL
			for i := 0; i < 6; i++ {
				if d.clkFreq[i] > (multiSynthShareMax * freqMult) {
					//if(i != clk && pll_assignment[i] == pll_assignment[clk]) {
					//	return  // won't set if any other clks already >100 MHz
					//}
				}
			}

			// Enable the output on first set_freq only
			if d.clkFirstSet[clk] == false {
				d.OutputEnable(clk, true)
				d.clkFirstSet[clk] = true
			}

			// Set the freq in memory
			d.clkFreq[clk] = freq

			fmt.Println("d.clk_freq: ", d.clkFreq)

			// Calculate the proper PLL frequency
			p1, p2, p3, pllFreq := d.CalcMultiSynth(int64(freq), 0)

			fmt.Println("MuS: - pll_freq: ", pllFreq, " p1,p2,p3:", p1, p2, p3)

			d.SetupPLLFreq(0, int32(pllFreq))

			//  Recalculate params for other synths on same PLL
			for i := 0; i < 6; i++ {
				if d.clkFreq[i] != 0 {
					// Select the proper R div value
					tempFreq := d.clkFreq[i]
					rDiv = d.selectRDiv(tempFreq)

					p1, p2, p3, pllFreq = d.CalcMultiSynth(int64(tempFreq), pllFreq)
					fmt.Println("MuS: after recalc: ", " p1,p2,p3:", p1, p2, p3)
					// If freq > 150 MHz, we need to use DIVBY4 and integer mode
					if tempFreq >= multisynthDivby4Freq*freqMult {
						divBy4 = 1
						intMode = true
					} else {
						divBy4 = 0
						intMode = false
					}

					// Set multisynth registers
					//set_ms((enum si5351_clock)i, temp_reg, int_mode, r_div, div_by_4);
					//d.SetupMultiSynth(int8(i),0,p1,p2,p3)
					d.SetupMultiSynthNew(int8(i), 0, p1, p2, p3, intMode, int32(rDiv), int32(divBy4))
				}
			}

			// TODO: делать согласно правильному асигменту
			// d.pllAssigment(clk)
			d.PLLReset(0)

		} else {
			// Set the freq in memory
			d.clkFreq[clk] = freq

			// Enable the output on first set_freq only
			if d.clkFirstSet[clk] == false {
				d.OutputEnable(clk, true)
				d.clkFirstSet[clk] = true
			}

			// Select the proper R div value
			rDiv = d.selectRDiv(freq)

			// Calculate the synth parameters
			//if(pll_assignment[clk] == SI5351_PLLA)
			//{
			//	multisynth_calc(freq, plla_freq, &ms_reg);
			p1, p2, p3, pllFreq := d.CalcMultiSynth(int64(freq), 0)
			d.SetupPLLFreq(0, int32(pllFreq))
			//}
			//else
			//{
			//	multisynth_calc(freq, pllb_freq, &ms_reg);
			//}

			//Set multisynth registers
			//set_ms(clk, ms_reg, int_mode, r_div, div_by_4);
			d.SetupMultiSynth(int8(clk), 0, p1, p2, p3)

		}

	} else {
		// MS6 and MS7 logic
		// not having to test
	}

}

func (d *Dev) SetupPLLFreq(targetPLL uint, pllFreq int32) {
	var p1, p2, p3 int64
	var baseAddr byte
	if targetPLL == PLLA {
		p1, p2, p3 = d.PLLCalc(PLLA, int64(pllFreq))
		baseAddr = pllAParameters

	} else if targetPLL == PLLB {
		p1, p2, p3 = d.PLLCalc(PLLA, int64(pllFreq))
		baseAddr = pllBParameters
	}

	//fmt.Println(" PLL freq: ", pllFreq, " p1,p2,p3:", p1,p2,p3 )

	// Set the main PLL config registers
	p1 = 128*p1 + int64(128.0*p2/p3) - 512
	p2 = 128*p2 - p3*int64(128.0*p2/p3)
	//p3 = p3

	buff := []byte{
		baseAddr,
		byte((p3 >> 8) & 0xFF),
		byte(p3 & 0xFF),
		byte((p1 >> 16) & 0x03),
		byte((p1 >> 8) & 0xFF),
		byte(p1 & 0xFF),
		byte((p3>>12)&0xF0 | (p2>>16)&0x0F),
		byte((p2 >> 8) & 0xFF),
		byte(p2 & 0xFF),
	}

	if err := d.d.Tx(buff, nil); err != nil {
		fmt.Println(err)
	}

	if targetPLL == PLLA {
		d.pllAFreq = pllFreq
	} else if targetPLL == PLLB {
		d.pllBFreq = pllFreq
	}

}

func (d *Dev) UpdateSysStatus() error {
	buff, err := d.Read(deviceStatus)
	if err != nil {
		return fmt.Errorf("can't update sys status: %v", err)
	}
	regValue := buff[0] // need only 1 byte

	// Parse the register value
	d.Status.sysInit = (int32(regValue) >> 7) & 0x01
	d.Status.lolB = (int32(regValue) >> 6) & 0x01
	d.Status.lolA = (int32(regValue) >> 5) & 0x01
	d.Status.los = (int32(regValue) >> 4) & 0x01
	d.Status.revID = int32(regValue) & 0x03

	return nil
}

func (d *Dev) UpdateIntStatus() error {

	buff, err := d.Read(interruptStatusSticky)
	if err != nil {
		return fmt.Errorf("can't update int status: %v", err)
	}

	regValue := buff[0] // need only 1 byte

	// Parse the register value
	d.Status.sysInitStky = (int32(regValue) >> 7) & 0x01
	d.Status.lolBStky = (int32(regValue) >> 6) & 0x01
	d.Status.lolAStky = (int32(regValue) >> 5) & 0x01
	d.Status.losStky = (int32(regValue) >> 4) & 0x01
	return nil

}

// Enable or disable a chosen output
func (d *Dev) OutputEnable(clk uint, status bool) {
	buff, _ := d.Read(outputEnableControl)
	regValue := buff[0] // need only 1 byte
	if status {
		regValue &= ^(1 << clk)
	} else {
		regValue |= 1 << clk
	}
	d.WriteCommand(outputEnableControl, regValue)
}

func (d *Dev) PLLCalc(pllNum int32, freq int64) (int64, int64, int64) {
	var p1, p2, p3 int64
	//correction := int64(0)

	ref_freq := d.crystalFreq * freqMult

	//// TODO: разобраться с лимитами
	//// PLL bounds checking
	//if (freq < pllVcoMin * freqMult) {
	//	freq = pllVcoMin * freqMult
	//}
	//
	//if (freq > pllVcoMax * freqMult) {
	//	freq = pllVcoMax * freqMult
	//}

	// Factor calibration value into nominal crystal frequency
	// Measured in parts-per-billion

	ref_freq = ref_freq + (((((int64(d.refCorrection)) << 31) / 1000000000) * ref_freq) >> 31)

	// Determine integer part of feedback equation
	p1 = int64(freq) / int64(ref_freq)

	// TODO: не выходим за пределы
	if p1 < pllAMin {
		freq = ref_freq * pllAMin
	}
	if p1 > pllAMax {
		freq = ref_freq * pllAMax
	}

	// Find best approximation for b/c = fVCO mod fIN
	// denom = 1000ULL * 1000ULL;
	// lltmp = freq % ref_freq;
	// lltmp *= denom;
	// do_div(lltmp, ref_freq);

	//b = (((uint64_t)(freq % ref_freq)) * rfracDenom) / ref_freq;

	//fmt.Println("ref freq",ref_freq)
	//fmt.Println("step1", int64(freq) % ref_freq )

	//num = (pll_freq - (uint64_t)mult*xtal_freq/10)*FRAC_DENOM*10/xtal_freq;
	p2 = ((int64(freq) - p1*ref_freq) * rfracDenom) / ref_freq
	if p2 > 0 {
		p3 = rfracDenom
	} else {
		p3 = 1
	}

	//// Recalculate frequency as fIN * (a + b/c)
	//lltmp := ref_freq;
	//lltmp *= b;
	//lltmp = lltmp / c;
	//dodiv(lltmp, c);
	//freq = lltmp;
	//freq += ref_freq * a;

	//fmt.Printf("p1: %v, p2: %v, p3: %v\n", p1, p2 , p3)
	return p1, p2, p3
}

func (d *Dev) CalcMultiSynth(freq, pllFreq int64) (int64, int64, int64, int64) {
	var p1, p2, p3, lltmp int64
	divby4 := 0

	// TODO: MultiSynth bounds checking
	if freq > multiSynthMaxFreq*freqMult {
		freq = multiSynthMaxFreq * freqMult
	}

	if freq < multiSynthMinFreq*freqMult {
		freq = multiSynthMinFreq * freqMult
	}

	if freq >= multisynthDivby4Freq*freqMult {
		divby4 = 1
	}

	if pllFreq == 0 {
		// Find largest integer divider for max
		// VCO frequency and given target frequency
		if divby4 == 0 {
			lltmp = pllVcoMax * freqMult // margin needed?
			lltmp = lltmp / freq
			if lltmp == 5 {
				lltmp = 4
			} else if lltmp == 7 {
				lltmp = 6
			}
			p1 = lltmp
		} else {
			p1 = 4
		}
		p2 = 0
		p3 = 1
		pllFreq = p1 * freq
	} else {
		// Preset PLL, so return the actual freq for these params instead of PLL freq
		//retVal := 1

		// Determine integer part of feedback equation
		p1 = pllFreq / freq

		if p1 < multisynthAMin {
			freq = pllFreq / multisynthAMin
		}
		if p1 > multisynthAMax {
			freq = pllFreq / multisynthAMax
		}

		p2 = (pllFreq % freq * rfracDenom) / freq
		//c := b ? rfracDenom : 1
		if p2 > 0 {
			p3 = rfracDenom
		} else {
			p3 = 1
		}
	}

	// Calculate parameters
	if divby4 == 1 {
		p3 = 1
		p2 = 0
		p1 = 0
	}

	return p1, p2, p3, pllFreq

}

func (d *Dev) EnableOutputs(status bool) {
	if status {
		d.WriteCommand(outputEnableControl, 0x00)
	} else {
		d.WriteCommand(outputEnableControl, 0xFF)
	}
}

func (d *Dev) selectRDiv(freq int32) int {
	rDiv := outputClkDiv1

	// Choose the correct R divider
	if (freq >= clkoutMinFreq*freqMult) && (freq < clkoutMinFreq*freqMult*2) {
		rDiv = outputClkDiv128
		freq *= 128
	} else if (freq >= clkoutMinFreq*freqMult*2) && (freq < clkoutMinFreq*freqMult*4) {
		rDiv = outputClkDiv64
		freq *= 64
	} else if (freq >= clkoutMinFreq*freqMult*4) && (freq < clkoutMinFreq*freqMult*8) {
		rDiv = outputClkDiv32
		freq *= 32
	} else if (freq >= clkoutMinFreq*freqMult*8) && (freq < clkoutMinFreq*freqMult*16) {
		rDiv = outputClkDiv16
		freq *= 16
	} else if (freq >= clkoutMinFreq*freqMult*16) && (freq < clkoutMinFreq*freqMult*32) {
		rDiv = outputClkDiv8
		freq *= 8
	} else if (freq >= clkoutMinFreq*freqMult*32) && (freq < clkoutMinFreq*freqMult*64) {
		rDiv = outputClkDiv4
		freq *= 4
	} else if (freq >= clkoutMinFreq*freqMult*64) && (freq < clkoutMinFreq*freqMult*128) {
		rDiv = outputClkDiv2
		freq *= 2
	}
	return rDiv
}

func (d *Dev) String() string {
	// d.dev.Conn
	return fmt.Sprintf("%s{%s}", d.name, d.d)
}

// Halt implements conn.Resource.
func (d *Dev) Halt() error {
	return nil
}

func (d *Dev) WriteCommand(reg, value byte) error {
	buf := []byte{reg, value}
	if err := d.d.Tx(buf, nil); err != nil {
		return d.wrap(err)
	}
	return nil
}

func (d *Dev) Read(reg byte) ([]byte, error) {
	buff := []byte{0}

	err := d.d.Tx([]byte{reg}, buff)
	if err != nil {
		return []byte{}, d.wrap(err)
	}
	return buff, nil
}

func (d *Dev) wrap(err error) error {
	return fmt.Errorf("%s: %v", strings.ToLower(d.name), err)
}
