`timescale 1ns / 1ps

//A lane executes only when its index is below VL AND its selected predicate bit is true.

//reset is required to have P[0] to all 1's.
//at reset all the values of VL go to 0, so vector length is 0, no lane is active.

//The the note of the above two.

module register_file #(
    parameter int DATA_WIDTH = 64,
    parameter int NUM_LANES  = 8,
) (
    input  logic                                        clk,
    input  logic                                        rst_n,

    input  logic [4:0]                                  raddr_a,  //Address of A 5 bits
    input  logic [4:0]                                  raddr_b,  //Address of B 5 bits
    input  logic [4:0]                                  raddr_c,  //Address of C 5 bits 
    input  logic [2:0]                                  pm_sel,   //Which predicate bit to be selected from P0 to P7
    //Each P0 to P7 are 8 bit values. P0[1].. to P0[Lane count] correspond to each of the lanes
    //Consider P0 to P7 as each new condition.. which has true and false things to it. 
    //P0 condition where pm_sel == 3'b0 is all 1's all the vector lanes are working 
    //Here is the function of each of ther predicate:
    //First 3 bits LSB of the SIMD isntructions define which out of the 8 predicate registers or conditions  

    output logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        rdata_a,    //register data for A
    output logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        rdata_b,    //register data for B
    output logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        rdata_c,    //register data for C

    //all the instruction types whose last 3 bits are allocated to select the predicate bits you will get rdata_pm
    //Not used in CONTROL Type instructions 
    output logic [NUM_LANES-1:0]                        rdata_pm,   //reads the predicate register which is selected by pm_sel out of P0 to P7
    
    //rdata_p_a, rdata_p_b, rdata_p_c is used for CONTROL Family and ALU.Select isntruction
    //for instructions CONTROL.PAND and CONTROL.PNOT
    output logic [NUM_LANES-1:0]                        rdata_p_a, //used in CONTROL.PAND and CONTROL.PNOT
    output logic [NUM_LANES-1:0]                        rdata_p_b, //used in CONTROL.PAND
    output logic [NUM_LANES-1:0]                        rdata_p_c, //used in ALU.SELECT
    
    //Current active vector length (VL) 
    output logic [31:0]                                 vl_out,
    //write desitnaton register adderss (waddr) for D
    input  logic [3:0]                                  waddr_d,  

    //We have considered 2 write registers D and C, C comes up in one of the div instructions giving reminder and quotient

    // Write register D and its signals
    input  logic                                        wren_v_d,   //enable the vector value write to D (vector reg) (to the address defined by waddr_d)
    input  logic                                        wren_s_d,   //enable the scalar value wrire to D (scalar reg) (to the address defined by waddr_d)
    input  logic                                        wren_p_d,   //yes predicate bit also has a write back lol CONTROL.PAND and CONTROL.PNOT
    input  logic [1:0]                                  wsize_d,    //Here the size of write back... check the size and add the input here.
    input  logic [NUM_LANES-1:0]                        lane_mask_d, //This is the AND of vector length and the predicate mask

    //Each lane putting out 64 bits so total of NUM_LANES*DATA_WIDTH is the length of the wdata_d
    //Note both Vector and Scalar will be taken care by this wdata_d
    input  logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        wdata_d,    //Here is the actual data that is to be stored
    input  logic [NUM_LANES-1:0]                        wdata_p_d,  //predicate NUM_LANES output.

    // Same things for C (Secondary write port for DIVMOD remainder)
    // We have not included predicate for this as 2 outputs are only for one instructions
    // All the instructions giving out the values for predicate is only 1 
    input  logic [3:0]                                  waddr_c,    
    input  logic                                        wren_v_c,
    input  logic                                        wren_s_c,
    input  logic [1:0]                                  wsize_c,
    input  logic [NUM_LANES-1:0][DATA_WIDTH-1:0]        wdata_c,

    //All related to VL and its change
    input  logic                                        setvl_en, //signal when we have change in setvl if not let the VL be the old val
    input  logic [31:0]                                 setvl_val, //value for the VL

    output logic                                        divmod_conflict //conflict in DIVMOD when we have the addr of D and C the same.
);

    //Creating the entire register file, everything read and written to these structure
    logic [NUM_LANES-1:0][DATA_WIDTH-1:0] v_reg [0:7];  //Defining the strcture of vector register
    logic [DATA_WIDTH-1:0]                s_reg [0:7];  //Defining the strcture of the scalar register
    logic [NUM_LANES-1:0]                 p_reg [0:7];  //Defining the strcture of the predicate register
    logic [31:0]                          vl_reg;       //Defining the strcture of the VL register.


    //defining the constants
    localparam logic [DATA_WIDTH-1:0] CONST_ZERO     = '0;  
    localparam logic [DATA_WIDTH-1:0] CONST_ALL_ONES = '1;
    localparam logic [DATA_WIDTH-1:0] CONST_ONE      = {{(DATA_WIDTH-1){1'b0}}, 1'b1};
    localparam logic [DATA_WIDTH-1:0] CONST_NEG_ZERO = {1'b1, {(DATA_WIDTH-1){1'b0}}};

    //These are brodcast values, this is specific for both scalar registers and constants.
    //Where one value is used for multiple lanes.
    logic [DATA_WIDTH-1:0] bval_a, bval_b, bval_c;

    //divmod_conflict checked.
    always_comb begin
        divmod_conflict = 1'b0;
        if ((wren_v_d && wren_v_c) && (waddr_d[2:0] == waddr_c[2:0]))
            divmod_conflict = 1'b1;
        if ((wren_s_d && wren_s_c) && (waddr_d[2:0] == waddr_c[2:0]))
            divmod_conflict = 1'b1;
    end

    //Write data here rather than storing garbage value for upper bits we replace them with 0's.
    //This is after the write is complete, the length is extended to 64 bits
    function automatic [DATA_WIDTH-1:0] format_wdata(
        input logic [DATA_WIDTH-1:0] data,
        input logic [1:0]            size
    );
        case (size)
            2'b00:   format_wdata = {{(DATA_WIDTH-8){1'b0}},  data[7:0]};
            2'b01:   format_wdata = {{(DATA_WIDTH-16){1'b0}}, data[15:0]};
            2'b10:   format_wdata = {{(DATA_WIDTH-32){1'b0}}, data[31:0]};
            default: format_wdata = data;
        endcase
    endfunction


    //Ouput of read function for scalar and vector registers based on the address given as the input to the reg
    //first three cases are for constants and scalar register
    //Val from 8 to 15 are scalar and 16 to 19 are the constants.
    //raddr_a[2:0] is used rather than raddr_a due to the fact the raddr_a is 5 bits can go till 31, if we check
    //31 value inside the s_reg or v_reg which is restricted to 3 bits, error occurs
    always_comb begin
        case (raddr_a)
            5'd8,  5'd9,  5'd10, 5'd11,
            5'd12, 5'd13, 5'd14, 5'd15: bval_a = s_reg[raddr_a[2:0]];
            5'd16:                      bval_a = CONST_ZERO;
            5'd17:                      bval_a = CONST_NEG_ZERO;
            5'd18:                      bval_a = CONST_ONE;
            5'd19:                      bval_a = CONST_ALL_ONES;
            default:                    bval_a = '0;
        endcase

        case (raddr_b)
            5'd8,  5'd9,  5'd10, 5'd11,
            5'd12, 5'd13, 5'd14, 5'd15: bval_b = s_reg[raddr_b[2:0]];
            5'd16:                      bval_b = CONST_ZERO;
            5'd17:                      bval_b = CONST_NEG_ZERO;
            5'd18:                      bval_b = CONST_ONE;
            5'd19:                      bval_b = CONST_ALL_ONES;
            default:                    bval_b = '0;
        endcase

        case (raddr_c)
            5'd8,  5'd9,  5'd10, 5'd11,
            5'd12, 5'd13, 5'd14, 5'd15: bval_c = s_reg[raddr_c[2:0]];
            5'd16:                      bval_c = CONST_ZERO;
            5'd17:                      bval_c = CONST_NEG_ZERO;
            5'd18:                      bval_c = CONST_ONE;
            5'd19:                      bval_c = CONST_ALL_ONES;
            default:                    bval_c = '0;
        endcase

        //Now comes for vector registers, if the address is between 0 to 7 (both included)
        //read data for vector reg, if the address is more than that, then take the scalar val or constant
        if (raddr_a <= 5'd7)
            rdata_a = v_reg[raddr_a[2:0]];
        else
            for (int i = 0; i < NUM_LANES; i++) rdata_a[i] = bval_a;

        if (raddr_b <= 5'd7)
            rdata_b = v_reg[raddr_b[2:0]];
        else
            for (int i = 0; i < NUM_LANES; i++) rdata_b[i] = bval_b;

        if (raddr_c <= 5'd7)
            rdata_c = v_reg[raddr_c[2:0]];
        else
            for (int i = 0; i < NUM_LANES; i++) rdata_c[i] = bval_c;
    end

    //Do the same for the predicate bits, for 2 to 3 instructions required.
    //is P0 is selected, it is always hardwired to 1, all bits 1
    always_comb begin
        rdata_pm = (pm_sel == 3'd0) ? {NUM_LANES{1'b1}} : p_reg[pm_sel];

        if (raddr_a <= 5'd7)
            rdata_p_a = (raddr_a[2:0] == 3'd0) ? {NUM_LANES{1'b1}} : p_reg[raddr_a[2:0]];
        else
            rdata_p_a = {NUM_LANES{1'b0}};

        if (raddr_b <= 5'd7)
            rdata_p_b = (raddr_b[2:0] == 3'd0) ? {NUM_LANES{1'b1}} : p_reg[raddr_b[2:0]];
        else
            rdata_p_b = {NUM_LANES{1'b0}};

        if (raddr_c <= 5'd7)
            rdata_p_c = (raddr_c[2:0] == 3'd0) ? {NUM_LANES{1'b1}} : p_reg[raddr_c[2:0]];
        else
            rdata_p_c = {NUM_LANES{1'b0}};
    end

    //VL value is pused out here from the vl_reg, both the same width.
    assign vl_out = vl_reg;

    //Now here is the structure for the register file to update the values
    always_ff @(posedge clk or negedge rst_n) begin
        //if reset is applied, all the vlaues are set to 0.
        if (!rst_n) begin
            for (int r = 0; r < 8; r++) begin
                v_reg[r] <= '0;
                s_reg[r] <= '0;
                p_reg[r] <= '0;
            end

            //P[0] is set to all 1's during reset as expected.
            p_reg[0] <= {NUM_LANES{1'b1}};
            //Initially all the values of VL registers are initalized to 0.
            vl_reg   <= '0;

        end 
        else begin
            //Write logic is written here
            p_reg[0] <= {NUM_LANES{1'b1}};

            //if write enable is true then 
            if (wren_v_d && !divmod_conflict) begin
                for (int lane = 0; lane < NUM_LANES; lane++) begin
                    if (lane_mask_d[lane]) //lane_mask_d AND of the VL and predicate bits so only which passes both the data is written to.
                        //extended 64 bit value written back to the v_reg.
                        v_reg[waddr_d[2:0]][lane] <= format_wdata(wdata_d[lane], wsize_d); 
                end
            end

            //scalar write back for destination register D
            if (wren_s_d && !divmod_conflict)
                s_reg[waddr_d[2:0]] <= format_wdata(wdata_d[0], wsize_d);

            //Predicate register updated 
            if (wren_p_d && !divmod_conflict && (waddr_d[2:0] != 3'd0)) begin
                for (int lane = 0; lane < NUM_LANES; lane++) begin
                    if (lane_mask_d[lane]) //lane_mask_d AND of the VL and predicate bits so only which passes both the data is written to.
                        p_reg[waddr_d[2:0]][lane] <= wdata_p_d[lane];
                end
            end

            //Vector for destination register C
            if (wren_v_c && !divmod_conflict) begin
                for (int lane = 0; lane < NUM_LANES; lane++) begin
                    if (lane_mask_d[lane]) //lane_mask_d AND of the VL and predicate bits so only which passes both the data is written to.
                        v_reg[waddr_c[2:0]][lane] <= format_wdata(wdata_c[lane], wsize_c);
                end
            end

            //Scalar write destination register C
            if (wren_s_c && !divmod_conflict)
                s_reg[waddr_c[2:0]] <= format_wdata(wdata_c[0], wsize_c);

            //Update the value for VL 
            //if the setvl_en is 1
            if (setvl_en) begin
                vl_reg <= setvl_val;
            end
        end
    end


endmodule
