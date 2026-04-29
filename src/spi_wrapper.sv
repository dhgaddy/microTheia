`default_nettype none
//just the spi module and control logic from system_package.sv
//purpose is to enable testing of the spi interface independent of voxel_bin_core
//also will help clean up system_package module

module spi_wrapper #(
    parameter int DATA_WIDTH = 32
)(
    input  wire clk,
    input  wire rst,
    // SPI pins: our chip is slave
    input  wire SCLK,
    input  wire CS,
    input  wire MOSI,
    output wire MISO,
    // received word stream into chip
    output logic [DATA_WIDTH-1:0] evt_word,
    output logic                  evt_word_valid,
    // classification result from chip
    input  wire [1:0] gesture,
    input  wire       gesture_valid,
    input  wire       gesture_confidence,
    // SPI status (from SPI module)
    output wire spi_ready
);

    logic [DATA_WIDTH-1:0] word_in;
    logic [DATA_WIDTH-1:0] word_out;
    //signals for the control logic
    logic processing_word;
    logic process_next_word;
    logic processing_word_d;
    logic CS_d, request_next, spi_abort_rst, spi_do_rst;
    assign spi_do_rst = rst | spi_abort_rst;

    logic [2:0] classification_output;

    spi_module #(
        .SPI_MASTER   (1'b0),
        .SPI_WORD_LEN (DATA_WIDTH)
    ) spi_slave (
        .master_clock      (clk),
        .SCLK_OUT          (),
        .SCLK_IN           (SCLK),
        .SS_OUT            (),
        .SS_IN             (CS),
        .OUTPUT_SIGNAL     (MISO),
        .processing_word   (processing_word),
        .process_next_word (process_next_word),
        .data_word_send    (word_out),
        .INPUT_SIGNAL      (MOSI),
        .data_word_recv    (word_in),
        .do_reset          (spi_do_rst), //not just rst, to enable dumping aborted words if CS goes high mid-transaction. IP holds onto partial words between transactions otherwise
        .is_ready          (spi_ready)
    );

always_ff @(posedge clk) begin
    if (rst) begin
        evt_word              <= '0;
        evt_word_valid        <= 1'b0;
        process_next_word     <= 1'b0;
        processing_word_d     <= 1'b0;
        classification_output <= '0;
        CS_d                  <= 1'b1;
        spi_abort_rst         <= 1'b0;
    end else begin
        CS_d <= CS;

        // pull down signals that should be pulsed for one cycle only after triggering
        evt_word_valid    <= 1'b0;
        process_next_word <= 1'b0;
        spi_abort_rst     <= 1'b0;

        // If CS rises while the SPI IP still processing, then master aborted
        // a partial word and we should dump it. resetting the SPI IP so the next transaction starts clean
        if (!CS_d && CS && processing_word) begin // if rising edge CS + currently processing
            spi_abort_rst     <= 1'b1;
            processing_word_d <= 1'b0;
        end else if (spi_abort_rst) begin
            processing_word_d <= 1'b0;
        end else begin
            processing_word_d <= processing_word;

            // detect word completion
            if (processing_word_d && !processing_word) begin
                evt_word       <= word_in;
                evt_word_valid <= 1'b1;
            end

            // request next word only during active CS-low transaction
            if (!processing_word && !CS) begin
                process_next_word <= 1'b1;
            end
        end
            
        if (gesture_valid) begin
            classification_output <= {gesture_confidence, gesture};
        end
    end
end
//a little cleaner than before
assign word_out = {
    classification_output,
    {(DATA_WIDTH - 3){1'b0}}
};

endmodule

`default_nettype wire