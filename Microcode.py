
from collections import defaultdict

# Microcode ROMs order is denoted by a letter A-F.
# They are arranged from MSB to LSB: ABCDEFH
# Note that this order doesn't correspond physical positions (and designators)
# Therefore starting bits are:
# A 48 MWK2
# B 40 MWF2
# C 32 MWH2
# D 24 MWL2
# E 16 MWM2
# F 8  MWJ2
# H 0  MWE2
class MicroCode(object):
    def __init__(self):
        with open('CodeROM.txt') as f:
            lines = f.readlines()
            self.code = [int(line, 16) for line in lines]
            self.visited = set()
            self.selects = defaultdict(int)

        self.entries = [0]
        
        # Parse opcode map and generate labels
        with open('CPU-6309.txt') as f:
            lines = f.readlines()
            opc_map = [int(line, 16) for line in lines]
            self.labels = {}
            for opcode, addr in enumerate(opc_map):
                # Two low nibbles come from AR, which comes from the map ROM;
                # the third nibble is hardcoded to 0x1 (see addr 102)
                addr += 0x100
                self.addLabel(addr, f'Op_{opcode:02x}')
                self.entries.append(addr)

    def addLabel(self, addr, text):
        if not addr in self.labels:
            self.labels[addr] = []
        self.labels[addr].append(text)

    def getBits(self, word, start, size):
        return (word >> start) & (~(-1 << size))

    def getNextNotVisited(self, addr):
        for i in range(addr, len(self.code)):
            if not i in self.visited:
                return i;
        return None

    def disassembleEntries(self):
        while len(self.entries) > 0:
            addr = self.entries.pop(0)

            # Do not print empty lines for already visited entries
            if addr in self.visited:
                continue
            while not (addr is None or addr in self.visited):
                self.visited.add(addr)
                addr = self.disassembleOne(addr, self.code[addr])
            print()

    def disassemble(self):
        self.printHeader()
        # First disassemble all known entry points, they can also have labels
        self.disassembleEntries()

        # Some jumps are dynamic and calculated at runtime, so we still have larde
        # portion of the code we haven't seen. Try to find the first unseen microword
        # and restart the disassembly from it. Repeat till we've been everywhere        
        addr = 1
        while True:
            addr = self.getNextNotVisited(addr)
            if addr is None:
                break
            self.addLabel(addr, f'Unknown_entry_{addr:03x}')
            self.entries.append(addr)
            self.disassembleEntries()
            # We have definitely visited 'addr', so resume from the next one
            addr = addr + 1
                
        print('Mux select distribution')
        for name, value in self.selects.items():
            print(f'{name:3x}: {value}')

    def disassembleOne(self, addr, word):
        if addr in self.labels:
            for l in self.labels[addr]:
                print(l + ':')
        if word == 0:
            print(f'{addr:3x}: unused')
            return None

        # DP bus control
        d2d3    = self.getBits(word, 0, 4)
        # F bus control
        e6      = self.getBits(word, 4, 3)
        # Write control
        k11     = self.getBits(word, 7, 3)
        # Bus control function
        h11     = self.getBits(word, 10, 3)
        e7      = self.getBits(word, 13, 2)
        # Sequencer jsr logic
        jsr     = self.getBits(word, 15, 1)
        # Din for sequencers
        dest    = self.getBits(word, 16, 11)
        # Stack control for sequencers
        fe      = self.getBits(word, 27, 1)
        pup     = self.getBits(word, 28, 1)
        # S1, S0 lines of sequencers are derived using some extra logic
        seq0_s  = self.getBits(word, 29, 2) ^ 3 # U_L6A, U_L6D        
        sh0     = self.getBits(word, 31, 1) ^ 1 # U_K6A
        mw_c40  = self.getBits(word, 32, 1)
        # Switch statement and condition selector
        case_   = self.getBits(word, 33, 1)
        # Note that 'cond' reuses middle 4 bits of 'dest'
        cond    = self.getBits(word, 20, 4)
        # ALU control
        aluSrc  = self.getBits(word, 34, 3)
        aluOp   = self.getBits(word, 37, 3)
        aluDest = self.getBits(word, 40, 3)
        aluB    = self.getBits(word, 43, 4)
        aluA    = self.getBits(word, 47, 4)
        # ALU.CARRY_IN and shift select
        u_f6    = self.getBits(word, 51, 2)
        # U_D101A, LSB access for 16-bit register
        r16_lsb = self.getBits(word, 53, 1)
        # Extra bit for sequencer control, see below
        mw_a6   = self.getBits(word, 54, 1)
        # Register bank selector (explicit or CPL)
        mw_a7   = self.getBits(word, 55, 1)

        # S inputs for sequencers
        s21  = mw_c40 ^ 1           # U_K6B
        s11  = mw_a6 & (mw_c40 ^ 1) # U_J6C, U_K6D, U_K6C
        s1s0 = (s21 << 9) | (sh0 << 8) | (s11 << 5) | (sh0 << 4) | seq0_s

        self.selects[s1s0] += 1

        seqCode, next = self.getSeqCode(addr + 1, dest, s1s0, fe, pup, case_, cond, jsr)
        dpBus   = self.getDPBus(d2d3, dest, mw_a7)
        aluCIn  = self.getALUCIn(u_f6)
        aluCode = self.getALUCode(aluSrc, aluOp, aluDest, aluA, aluB, u_f6)
        bus     = self.getBusCtl(h11)
        extra   = self.getBusExtra(e7)
        fBus    = self.getFBus(e6, dest)
        write   = self.getWriteControl(k11, aluB, mw_a7)
        msb     = 'LSB' if r16_lsb else ''

        print(f'{addr:3x}: {dpBus:19s} {aluCIn:9s} {aluCode:40s} {fBus:51s} {bus:11s} {extra:11s} {write:20s} {msb:7s} {seqCode}')
        return next

    def printHeader(self):
        # Reuse formatting string from above, this makes it easier to adjust it
        addr    = 'Addr'
        dpBus   = 'DP'
        aluCIn  = 'ALUCIn'
        aluCode = 'ALUOp'
        fBus    = 'F'
        bus     = 'BusControl'
        extra   = 'BusExtra'
        write   = 'WriteCtl'
        msb     = 'R16_LSB'
        seqCode = 'Seq'
        print(f'{addr:4s} {dpBus:19s} {aluCIn:9s} {aluCode:40s} {fBus:51s} {bus:11s} {extra:11s} {write:20s} {msb:7s} {seqCode}')


    # Collect our switch targets, except #0, and insert them into todo list
    # Insertion happens in the beginning in reverse order, so that branches would be
    # disassembled immediately after our routine ends
    def getSwitchTargets(self, target, step):
        for i in range(0, 3):
            # Reverse our range: 3 2 1
            # Skip 0 because our caller will try to continue from there
            i = 3 - i
            addr = target|(step * i)
            self.entries.insert(0, addr)

    def getRange(self, base, mask):
        targets = set()
        for i in range(0, len(self.code)):
            addr = i & mask
            targets.add(base | addr)
        for addr in sorted(targets):
            self.entries.append(addr)

    def getSeqCode(self, next, dest, s1s0, fe, pup, case_, cond, jsr):
        if jsr == 0:
            JSR_COND = ['Cycle', 'RegIdx & 0x11 == 0', 'RegIdx & 1', 'REG_MMIO', 'RegOrPageOut', 'DMARequest', 'MemFault', 'MultiINT']
            cond = JSR_COND[dest & 7]
            jsr_ = f'if {cond} jsr {dest:x}'
            self.entries.append(dest)
            if s1s0 == 0x000:
                return jsr_, next
            jsr_ += ' else '
        else:
            jsr_ = ''

        if fe == 0 and pup == 1:
            push = f'push {next:x}'
        else:
            push = ''

        # Some shorthands
        # jump AR isn't really supported because AR isn't used on the third sequencer;
        # it's AE line is permanently tied high.
        if s1s0 == 0x000 and case_ == 1:
            jump = '' # jump uPC (next)
        elif s1s0 == 0x222 and case_ == 1:
            if fe == 0 and pup == 0 :            
                jump = 'ret' # jump STK0; pop
            else:
                jump = 'jump STK0'
            next = None
        else:
            ar_mask    = 0;
            pop_mask   = 0;
            const_mask = 0;
            target     = 0;

            for i in range(0, 3):
                bits = (s1s0 >> (i * 4)) & 3
                mask = 0xf << (i * 4)
                if bits == 0:
                    target |= next & mask
                    const_mask |= mask
                elif bits == 1:
                    ar_mask |= mask
                elif bits == 2:
                    pop_mask |= mask
                else: # bits == 3
                    target |= dest & mask
                    const_mask |= mask

            if case_ == 1:
                if push != '':
                    jump = f'jsr {target:x}' # push; jmp
                    push = ''
                    self.entries.append(target)
                else:
                    jump = f'jump {target:x}'
                    next = target
            elif const_mask & 0x00f == 0:
                # Since the switch is OR-based, we need the less significant 4 bits
                # to have a known base value, so we always take them from a constant
                jump += ' !WARN unexpected switch'
            # The first 4 branches use OR lines 0 and 1 via U_J13
            elif cond == 0b1100:
                jump = f'switch flags(ZM) jump ({target|0:x}, {target|1:x}, {target|2:x}, {target|3:x})'
                self.getSwitchTargets(target, 1)
                next = target
            elif cond == 0b1101:
                jump = f'switch flags(VH) jump ({target|0:x}, {target|1:x}, {target|2:x}, {target|3:x})'
                self.getSwitchTargets(target, 1)
                next = target
            elif cond == 0b1110:
                jump = f'switch pagetable??? jump ({target|0:x}, {target|1:x}, {target|2:x}, {target|3:x})'
                self.getSwitchTargets(target, 1)
                next = target
            # These branches use OR lines 2 and 3 via U_K13
            elif cond == 0b0011:
                jump = f'switch flags(IL) jump ({target|0:x}, {target|4:x}, {target|8:x}, {target|12:x})'
                self.getSwitchTargets(target, 4)
                next = target
            elif cond == 0b0111:
                jump = f'switch interrupts??? jump ({target|0:x}, {target|4:x}, {target|8:x}, {target|12:x})'
                self.getSwitchTargets(target, 4)
                next = target
            elif cond == 0b1011:
                jump = f'switch dma??? jump ({target|0:x}, {target|4:x}, {target|8:x}, {target|12:x})'
                self.getSwitchTargets(target, 4)
                next = target
            # Combined switches don't make sense
            else:
                jump += f'!WARN bad switch {cond:x}'

            if pop_mask != 0:
                jump += f'|(STK0 & {pop_mask:x})'
#                self.getRange(next, pop_mask)
                next = None
            if ar_mask != 0:
                jump += f'|(SAR & {ar_mask:x})'
#                self.getRange(next, ar_mask)
                next = None

            if fe == 0 and pup == 0:
                jump += '; pop'

        op = jsr_ + push
        if op == '':
            return jump, next
        if jump == '':
            return op, next
        sep = '; ' if push else ''
        return op + sep + jump, next

    def getDPBus(self, d2d3, dest, mw_a7):
        # 'dest' is also used for constants, but these are inverted
        constant = ~dest & 0xff
        if d2d3 == 0:
            return 'READ.SWP'
        elif d2d3 == 1:
            return 'READ.' + self.getRegName(mw_a7)
        elif d2d3 == 2:
            return 'READ.MAR.H'
        elif d2d3 == 3:
            return 'READ.MAR.L'
        elif d2d3 == 4:
            return 'READ.SWP  '
        elif d2d3 == 5:
            return 'READ.' + self.getRegName(mw_a7)
        elif d2d3 == 6:
            return 'READ.MAR.H'
        elif d2d3 == 7:
            return 'READ.MAR.L'
        elif d2d3 == 8:
            return 'READ.PF'
        elif d2d3 == 9:
            return 'READ.CCR'
        elif d2d3 == 10:
            return 'READ.D.BUS'
        elif d2d3 == 11:
            return 'READ.MSR'
        elif d2d3 == 12:
            return 'READ.INR'
        elif d2d3 == 13:
            return f'READ.CONST:{constant:02x}'
        elif d2d3 == 14:
            return ''
        elif d2d3 == 15:
            return ''

    def getALUCode(self, aluSrc, aluOp, aluDest, aluA, aluB, u_f6):
        ALU_SRC_MAP = [['A', 'Q'], ['A', 'B'], ['0', 'Q'], ['0', 'B'], ['0', 'A'], ['D', 'A'], ['D', 'Q'], ['D', '0']]
        ALU_OP_MAP = ['{r}+{s}', '{s}-{r}', '{r}-{s}', '{r}|{s}', '{r}&{s}', '(~{r})&{s}', '{r}^{s}', '~({r}^{s})']
        ALU_MEM_DEST_MAP = [''     , ''     , 'r{b}={f}', 'r{b}={f}', 'r{b}=({f})>>1', 'r{b}=({f})>>1', 'r{b}=({f})<<1', 'r{b} =({f})<<1']
        ALU_Q_DEST_MAP   = ['Q={f}', ''     , ''        , ''        , 'Q>>=1'        , ''             , 'Q<<=1'        , ''              ]
        ALU_OUT_MAP      = ['Y={f}', 'Y={f}', 'Y={a}'   , 'Y={f}'   , 'Y={f}'        , 'Y={f}'        , 'Y={f}'        , 'Y={f}'         ]

        cin = 0
        cout = 0
        r, s = ALU_SRC_MAP[aluSrc]
        if r == 'A':
            r = f'r{aluA}'
        elif r == 'B':
            r = f'r{aluB}'
        if s == 'A':
            s = f'r{aluA}'
        elif s == 'B':
            s = f'r{aluB}'
        f = ALU_OP_MAP[aluOp].format(r=r, s=s)
        mem = ALU_MEM_DEST_MAP[aluDest].format(b=aluB, f=f)
        q = ALU_Q_DEST_MAP[aluDest].format(f=f)
        a = f'r{aluA}'
        y = ALU_OUT_MAP[aluDest].format(f=f, a=a)
        c = ''
        if cout:
            c = 'C'
        if (aluOp == 0 or aluOp == 1 or aluOp == 2) and cin:
            str = f'{mem}+{cin} {q} {y}+{cin} {c}'
        else:
            str = f'{mem} {q} {y} {c}'

        str = str.strip()

        if aluDest >= 4:
            str += ' ' + self.getShiftSel(aluDest & 2, u_f6)
            
        return str

    # U_E6
    def getFBus(self, val, dest):
        if val == 7:
            # CCR write uses 'dest' as flag selector
            sz_sel       = self.getBits(dest, 0, 2)
            fault_enable = self.getBits(dest, 2, 1)
            fault_sel    = self.getBits(dest, 3, 2)
            link_enable  = self.getBits(dest, 5, 1)
            link_sel     = self.getBits(dest, 6, 3)

            sign  = self.getSignSel(sz_sel)
            zero  = self.getZeroSel(sz_sel)
            link  = self.getLinkSel(link_enable, link_sel)
            fault = self.getFaultSel(fault_enable, fault_sel)

            return 'LOAD.CCR{' + f'V={zero},M={sign},F={fault},L={link}' + '}'

        RESULT_MAP = ['        ', "LOAD.RR ", "LOAD.RIR", "LOAD.ILR", "LOAD.MAP", "LOAD.MAR", "LOAD.SAR"]
        return RESULT_MAP[val]

    # U_F6
    def getALUCIn(self, u_f6):
        CARRY_MAP = ['0', '1', 'FLR.C', '0']
        return CARRY_MAP[u_f6]

    # U_H6
    def getShiftSel(self, shift_up, u_f6):
        if shift_up:
            # Left shift, select which signal will be shifted in at LSB
            SHIFT_Q0_MAP = ['0', 'FLR.C', 'ALU.S', '1']
            val = SHIFT_Q0_MAP[u_f6]
            signal = 'Q0'
        else:
            # Right shift, select which signal will be shifted in at MSB
            SHIFT_RAM7_MAP = ['ALU.S', 'FLR.C', 'ALU.Q0', 'ALU.C']
            val = SHIFT_RAM7_MAP[u_f6]
            signal = 'RAM7'
        return f'{signal}={val}'

    # U_J12.a
    def getSignSel(self, sel):
        SIGN_TABLE = ['CCR.M', 'FLR.S', 'RR.D6', 'FLR.S']
        return SIGN_TABLE[sel]

    # U_J12.b
    def getZeroSel(self, sel):
        ZERO_TABLE = ['CCR.V', 'FLR.Z', 'RR.D7', 'FLR.Z&FLR.LZ']
        return ZERO_TABLE[sel]

    # U_J10
    def getLinkSel(self, enable, sel):
        if enable:
            return '0'
        else:
            LINK_TABLE = ['CCR.L', '/CCR.L', 'FLR.C', '1', 'RR.D4', 'ALU.SHIFT_RAM7', 'ALU_SHIFT_RAM0_Q7', 'ALU.SHIFT_Q0']
            return LINK_TABLE[sel]

    #U_J11
    def getFaultSel(self, enable, sel):
        if enable:
            return '0'
        else:
            FAULT_TABLE = ['RR.D5', '1', 'CCR.F', 'FLR.OVER']
            return FAULT_TABLE[sel]
        
    # U_C14
    def getRegName(self, mw_a7):
        if mw_a7:
            return 'RF[RIR.L][ILR]'
        else:
            return 'RF[RIR]'

    # U_H11
    def getBusCtl(self, h11):
        BUS_MAP = ['', 'BUS.RD', 'BUS.WT', 'LOAD.WAR.H', 'INC.WAR', 'INC.MAR', 'MAPROM.EN', 'LOAD.SWP']
        return BUS_MAP[h11]

    # U_E7
    def getBusExtra(self, u_e7):
        EXTRA_F_MAP = ['', 'BUS.ABT', 'LOAD.FLR', 'BUS.WAIT']
        return EXTRA_F_MAP[u_e7]

    # U_K11, U_K12C, U_H13B
    def getWriteControl(self, k11, aluB, mw_a7):
        if k11 == 2: # U_M13
            M13_MAP = ['CTL0.DMA', 'CTL1.DMA', 'TIMER', 'MAPROM.CE1', 'RUN', '/TIMER.RES', 'ABORT', 'IACK']
            return self.getDemuxedControl(M13_MAP, aluB)
        if k11 == 3: # U_F11
            F11_MAP = ['INT', '/ABE', '/INC.DMA', 'DIR', '/DMA', 'PARO', 'PE.EN', 'DMA.EN']
            return self.getDemuxedControl(F11_MAP, aluB)
        if k11 == 4: # Write from result register to register file
            reg = self.getRegName(mw_a7)
            return f'WRITE.{reg}'

        # M13, F11 and REGS are handled above
        WRITE_CONTROL = ['', 'DMA.RESET', 'M13.EN?', 'F11.EN?', 'WRITE.RF', 'WRITE.PF', 'LOAD.WAR.L', 'LOAD.DBR']
        # TODO: BusCtl uses numeric value from aluB (U_F11)
        return WRITE_CONTROL[k11]

    def getDemuxedControl(self, map_, aluB):
        bit  = '+' if aluB & 1 == 1 else '-'
        line = map_[(aluB >> 1) & 7]
        return f'{line}{bit}'

if __name__ == '__main__':
    mc = MicroCode()
    mc.disassemble()
