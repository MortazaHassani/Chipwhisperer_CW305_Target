// cw305_user_defines.v
// Register address constants.
// Width matches reg_address: pADDR_WIDTH - pBYTECNT_SIZE = 21 - 7 = 14 bits.
// slurp_defines() in CW305.py parses every line matching: define REG_*

`define REG_OPERAND_A    14'd0
`define REG_OPERAND_B    14'd1
`define REG_RESULT_LO    14'd2
`define REG_RESULT_HI    14'd3
`define REG_CRYPT_GO     14'd4
`define REG_USER_LED     14'd5
