
from collections import defaultdict

ALU_SRC_MAP = [['A', 'Q'], ['A', 'B'], ['0', 'Q'], ['0', 'B'], ['0', 'A'], ['D', 'A'], ['D', 'Q'], ['D', '0']]
ALU_OP_MAP = ['{r}+{s}', '{s}-{r}', '{r}-{s}', '{r}|{s}', '{r}&{s}', '(~{r})&{s}', '{r}^{s}', '~({r}^{s})']
ALU_MEM_DEST_MAP = ['', '', 'r{b}={f}', 'r{b}={f}', 'r{b}=({f})>>1', 'r{b}=({f})>>1', 'r{b}=({f})<<1', 'r{b} =({f})<<1']
ALU_Q_DEST_MAP = ['Q={f}', ''     , ''     , ''     , 'Q>>=1', ''     , 'Q<<=1', '']
ALU_OUT_MAP    = ['Y={f}', 'Y={f}', 'Y={a}', 'Y={f}', 'Y={f}', 'Y={f}', 'Y={f}', 'Y={f}']
RESULT_MAP = ['', 'Result', 'RIndex', 'Level', 'PTIndex', 'Swap', 'AR', 'CC']

# Microcode ROMs order is denoted by a letter A-F.
# They are arranged from MSB to LSB: ABCDEFH
# Therefore starting bits are:
# A 48
# B 40
# C 32
# D 24
# E 16
# F 8
# H 0
class MicroCode(object):
    def __init__(self):
        with open('CodeROM.txt') as f:
            lines = f.readlines()
            self.code = [int(line, 16) for line in lines]
            self.selects = defaultdict(int)

    def getBits(self, word, start, size):
        return (word >> start) & (~(-1 << size))

    def disassemble(self):
        print('Addr DP         ALU Op                           F         Result Sel   Seq Op')
        for addr, word in enumerate(self.code):
            self.disassembleOne(addr, word)
        print()
        print('Mux select distribution')
        for name, value in self.selects.items():
            print(f'{name:3x}: {value}')

    def disassembleOne(self, addr, word):
        if word == 0:
            print(f'{addr:3x} {word:13x} unused')
            return
        seq0_din = self.getBits(word, 16, 4)
        seq1_din = self.getBits(word, 20, 4)
        seq2_din = self.getBits(word, 24, 3)
        # S1, S0 lines of sequencers are derived using some extra logic
        seq0_s = self.getBits(word, 29, 2) ^ 3 # U_L6A, U_L6D        
        sh0    = self.getBits(word, 31, 1) ^ 1 # U_K6A
        mw_c40 = self.getBits(word, 32, 1)
        s21    = mw_c40 ^ 1                    # U_K6B
        mw_a6  = self.getBits(word, 54, 1)
        s11    = mw_a6 & (mw_c40 ^ 1) # U_J6C, U_K6D, U_K6C
        # Switch statement and condition selector
        case_ = self.getBits(word, 33, 1)
        cond = self.getBits(word, 20, 4)
        # Din for sequencers
        dest = (seq2_din << 8) | (seq1_din << 4) | seq0_din
        # S inputs for sequencers
        s1s0 = (s21 << 9) | (sh0 << 8) | (s11 << 5) | (sh0 << 4) | seq0_s
        # Stack control for sequencers
        fe  = self.getBits(word, 27, 1)
        pup = self.getBits(word, 28, 1)

        self.selects[s1s0] += 1
        next = addr + 1

        if fe == 0 and pup == 1:
            push = f'push {next:x}; '
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
                jump = f'jump {target:x}'
            elif const_mask & 0x00f == 0:
                # Since the switch is OR-based, we need the less significant 4 bits
                # to have a known base value, so we always take them from a constant
                jump += ' !WARN unexpected switch'
            # The first 4 branches use OR lines 0 and 1 via U_J13
            elif cond == 0b1100:
                jump = f'switch flags(ZM) jump ({target|0:x}, {target|1:x}, {target|2:x}, {target|3:x})'
            elif cond == 0b1101:
                jump = f'switch flags(VH) jump ({target|0:x}, {target|1:x}, {target|2:x}, {target|3:x})'
            elif cond == 0b1110:
                jump = f'switch pagetable??? jump ({target|0:x}, {target|1:x}, {target|2:x}, {target|3:x})'
            # These branches use OR lines 2 and 3 via U_K13
            elif cond == 0b0011:
                jump = f'switch flags(IL) jump ({target|0:x}, {target|4:x}, {target|8:x}, {target|12:x})'
            elif cond == 0b0111:
                jump = f'switch interrupts??? jump ({target|0:x}, {target|4:x}, {target|8:x}, {target|12:x})'
            elif cond == 0b1011:
                jump = f'switch dma??? jump ({target|0:x}, {target|4:x}, {target|8:x}, {target|12:x})'
            # Combined switches don't make sense
            else:
                jump += f'!WARN bad switch {cond:x}'

            if pop_mask != 0:
                jump += f'|(STK0 & {pop_mask:x})'
            if ar_mask != 0:
                jump += f'|(AR & {ar_mask:x})'

            if fe == 0 and pup == 0:
                jump += '; pop'

        seq_op = push + jump
        dpBus = self.getDPBus(word)
        aluOp = self.getALUCode(word)
        fBus = self.getFBus(word)
        result = self.getResult(word)
        print(f'{addr:3x}: {dpBus:10s} {aluOp:32s} {fBus:9s} {result:12s} {seq_op}')

    def getDPBus(self, word):
        d2d3 = self.getBits(word, 0, 4)
        constant = ~self.getBits(word, 16, 8) & 0xff
        if d2d3 == 0:
            return f'swap'
        elif d2d3 == 1:
            return f'reg_ram'
        elif d2d3 == 2:
            return f'mar_hi'
        elif d2d3 == 3:
            return f'mar_lo'
        elif d2d3 == 4:
            return f'swap'
        elif d2d3 == 5:
            return f'reg_ram'
        elif d2d3 == 6:
            return f'mar_hi'
        elif d2d3 == 7:
            return f'mar_lo'
        elif d2d3 == 8:
            return ''
        elif d2d3 == 9:
            return 'CC'
        elif d2d3 == 10:
            return 'bus_read'
        elif d2d3 == 11:
            return 'ILR?'
        elif d2d3 == 12:
            return 'dips?'
        elif d2d3 == 13:
            return f'const:{constant:x}'
        elif d2d3 == 14:
            return ''
        elif d2d3 == 15:
            return ''

    def getFBus(self, word):
        h11 = self.getBits(word, 10, 3)
        if h11 == 6:
            return 'map_rom'
        return 'Y'

    def getALUCode(self, word):
        aluA = self.getBits(word, 47, 4)
        aluB = self.getBits(word, 43, 4)
        aluSrc = self.getBits(word, 34, 3)
        aluOp = self.getBits(word, 37, 3)
        aluDest = self.getBits(word, 40, 3)
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
        return str.strip()

    def getResult(self, word):
        result_sel = self.getBits(word, 4, 3)
        # Result select decoder U_E6
        return RESULT_MAP[result_sel]

if __name__ == '__main__':
    mc = MicroCode()
    mc.disassemble()
