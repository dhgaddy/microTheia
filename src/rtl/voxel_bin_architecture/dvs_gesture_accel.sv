
// DVS Gesture Accelerator - Asynchronous Spatiotemporal Motion-Energy Classifier

module dvs_gesture_accel #(
    parameter CLK_FREQ_HZ        = 12_000_000,
    parameter WINDOW_MS          = 400,
    parameter GRID_SIZE          = 16,
    parameter SENSOR_RES         = 320,
    parameter FIFO_DEPTH         = 16,
    parameter MIN_EVENT_THRESH   = 20,
    parameter MOTION_THRESH      = 8,
    parameter PERSISTENCE_COUNT  = 2,
    parameter CYCLES_PER_BIN     = 0  // 0 = auto-compute from WINDOW_MS/NUM_BINS
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        event_valid,
    input  logic [8:0]  event_x,
    input  logic [8:0]  event_y,
    input  logic        event_polarity,
    input  logic [15:0] event_ts,
    output logic        event_ready,
    output logic [1:0]  gesture,
    output logic        gesture_valid,
    output logic [3:0]  gesture_confidence,
    output logic [7:0]  debug_event_count,
    output logic [2:0]  debug_state,
    output logic        debug_fifo_empty,
    output logic        debug_fifo_full,
    output logic        debug_temporal_phase
);

    localparam integer GRID_BITS     = $clog2(GRID_SIZE);
    localparam integer FIFO_PTR_BITS = $clog2(FIFO_DEPTH);

    logic                      fifo_pop;
    logic                      fifo_empty;
    logic                      fifo_full;
    logic [8:0]                fifo_out_x;
    logic [8:0]                fifo_out_y;
    logic                      fifo_out_pol;
    logic [15:0]               fifo_out_ts;

    logic signed [GRID_BITS:0] compressed_x;
    logic signed [GRID_BITS:0] compressed_y;
    logic                      compressed_pol;
    logic                      compressed_valid;

    localparam PARALLEL_READS = 4;
    localparam COUNTER_BITS   = 6;

    logic        readout_start;
    logic [PARALLEL_READS*COUNTER_BITS-1:0] readout_data;
    logic        readout_valid;

    localparam NUM_CLASSES = 4;
    localparam NUM_BINS    = 4;
    localparam NUM_CELLS   = NUM_BINS * GRID_SIZE * GRID_SIZE;
    localparam WEIGHT_BITS = 8;
    localparam ACC_BITS    = 24;

    logic [PARALLEL_READS*$clog2(NUM_CELLS)-1:0]       w_addr_flat;
    logic [PARALLEL_READS*NUM_CLASSES*WEIGHT_BITS-1:0] w_data_flat;

    logic [$clog2(NUM_CELLS)-1:0]       w_addr [0:PARALLEL_READS-1];
    logic [NUM_CLASSES*WEIGHT_BITS-1:0] w_data [0:PARALLEL_READS-1];
    logic                               sys_result_valid;
    logic [1:0]                         sys_best_class;
    logic [NUM_CLASSES*ACC_BITS-1:0]    sys_scores_flat;

    InputFIFO #(
        .DEPTH(FIFO_DEPTH),
        .PTR_BITS(FIFO_PTR_BITS)
    ) u_input_fifo (
        .clk           (clk),
        .rst           (rst),
        .push_valid    (event_valid),
        .push_x        (event_x),
        .push_y        (event_y),
        .push_polarity (event_polarity),
        .push_ts       (event_ts),
        .push_ready    (event_ready),
        .pop_req       (fifo_pop),
        .pop_x         (fifo_out_x),
        .pop_y         (fifo_out_y),
        .pop_polarity  (fifo_out_pol),
        .pop_ts        (fifo_out_ts),
        .empty         (fifo_empty),
        .full          (fifo_full)
    );

    assign debug_fifo_empty    = fifo_empty;
    assign debug_fifo_full     = fifo_full;
    assign debug_temporal_phase = 1'b0;

    SpatialCompressor #(
        .SENSOR_RES(SENSOR_RES),
        .GRID_SIZE (GRID_SIZE),
        .GRID_BITS (GRID_BITS)
    ) u_spatial_compressor (
        .clk         (clk),
        .rst         (rst),
        .in_valid    (!fifo_empty),
        .in_x        (fifo_out_x),
        .in_y        (fifo_out_y),
        .in_polarity (fifo_out_pol),
        .in_ready    (fifo_pop),
        .out_x       (compressed_x),
        .out_y       (compressed_y),
        .out_polarity(compressed_pol),
        .out_valid   (compressed_valid)
    );

    TimeSurfaceBinning #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .WINDOW_MS     (WINDOW_MS),
        .NUM_BINS      (NUM_BINS),
        .READOUT_BINS  (NUM_BINS),
        .GRID_SIZE     (GRID_SIZE),
        .COUNTER_BITS  (COUNTER_BITS),
        .PARALLEL_READS(PARALLEL_READS),
        .CYCLES_PER_BIN(CYCLES_PER_BIN)
    ) u_time_surface_binning (
        .clk           (clk),
        .rst           (rst),
        .event_valid   (compressed_valid),
        .event_x       (compressed_x),
        .event_y       (compressed_y),
        .event_polarity(compressed_pol),
        .readout_start (readout_start),
        .readout_data  (readout_data),
        .readout_valid (readout_valid)
    );

    assign debug_event_count = 8'd0;

    SystolicMatrixMultiply #(
        .NUM_CLASSES    (NUM_CLASSES),
        .NUM_CELLS      (NUM_CELLS),
        .VALUE_BITS     (COUNTER_BITS),
        .WEIGHT_BITS    (WEIGHT_BITS),
        .ACC_BITS       (ACC_BITS),
        .PARALLEL_INPUTS(PARALLEL_READS)
    ) u_systolic_array (
        .clk          (clk),
        .rst          (rst),
        .start        (readout_start),
        .feature_in   (readout_data),
        .feature_valid(readout_valid),
        .w_addr_flat  (w_addr_flat),
        .w_data_flat  (w_data_flat),
        .result_valid (sys_result_valid),
        .best_class   (sys_best_class),
        .scores_flat  (sys_scores_flat)
    );

    generate
        genvar pa;
        for (pa = 0; pa < PARALLEL_READS; pa = pa + 1) begin : gen_unpack_addr
            assign w_addr[pa] = w_addr_flat[(pa+1)*$clog2(NUM_CELLS)-1 : pa*$clog2(NUM_CELLS)];
        end
    endgenerate

    generate
        genvar p, k;
        for (p = 0; p < PARALLEL_READS; p = p + 1) begin : gen_parallel_roms
            for (k = 0; k < NUM_CLASSES; k = k + 1) begin : gen_class_roms
                WeightROM #(
                    .CLASS_IDX  (k),
                    .NUM_CELLS  (NUM_CELLS),
                    .GRID_SIZE  (GRID_SIZE),
                    .WEIGHT_BITS(WEIGHT_BITS)
                ) u_weight_rom (
                    .clk      (clk),
                    .cell_addr(w_addr[p]),
                    .dout     (w_data[p][(k+1)*WEIGHT_BITS-1 : k*WEIGHT_BITS])
                );
            end
        end
    endgenerate

    generate
        genvar pd;
        for (pd = 0; pd < PARALLEL_READS; pd = pd + 1) begin : gen_pack_data
            assign w_data_flat[(pd+1)*NUM_CLASSES*WEIGHT_BITS-1 : pd*NUM_CLASSES*WEIGHT_BITS] = w_data[pd];
        end
    endgenerate

    logic signed [ACC_BITS-1:0] sys_scores [0:NUM_CLASSES-1];
    generate
        genvar gi;
        for (gi = 0; gi < NUM_CLASSES; gi = gi + 1) begin : gen_unpack_sys_scores
            assign sys_scores[gi] = sys_scores_flat[(gi+1)*ACC_BITS-1 : gi*ACC_BITS];
        end
    endgenerate

    logic signed [ACC_BITS-1:0] best_score;
    logic [ACC_BITS-1:0]        abs_best_score;
    logic                       score_above_thresh;

    // Minimum score threshold: prevents noise from triggering gestures.
    // With 35 gesture events, expected score ~50-200.
    // With 5 random events, expected score <20.
    localparam integer MIN_SCORE_THRESH = 30;

    always_comb begin
        best_score = sys_scores[sys_best_class];

        if (best_score < 0)
            abs_best_score = -best_score;
        else
            abs_best_score = best_score;

        score_above_thresh = (abs_best_score >= MIN_SCORE_THRESH);
    end

    logic [17:0] pseudo_mag_x, pseudo_mag_y;
    assign pseudo_mag_x = {2'b0, abs_best_score[15:0]};
    assign pseudo_mag_y = 18'd0;

    OutputRegister #(
        .ACC_SUM_BITS     (18),
        .PERSISTENCE_COUNT(PERSISTENCE_COUNT)
    ) u_output_register (
        .clk              (clk),
        .rst              (rst),
        .class_gesture    (sys_best_class),
        .class_valid      (sys_result_valid),
        .class_pass       (sys_result_valid && score_above_thresh),
        .abs_delta_x      (pseudo_mag_x),
        .abs_delta_y      (pseudo_mag_y),
        .gesture          (gesture),
        .gesture_valid    (gesture_valid),
        .gesture_confidence(gesture_confidence),
        .debug_state      (debug_state)
    );

endmodule
