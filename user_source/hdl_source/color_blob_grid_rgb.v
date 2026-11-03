`ifndef COLOR_BLOB_GRID_RGB_V
`define COLOR_BLOB_GRID_RGB_V
// Statistics-only grid component selector. RGB/timing never passes through it.
// Pixels retain their exact local extrema; emitted bounds are not cell-rounded.
// Strong-core four-connected components include one nonrecursive weak halo.
// Only if no qualified core component exists, a separate weak-cell BFS runs.
// The selected center is the bounding-box midpoint, NOT a pixel centroid.
module color_blob_grid_rgb #(
    parameter integer IMG_WIDTH=1280,
    parameter integer IMG_HEIGHT=720,
    parameter integer SUM_WIDTH=32,
    parameter integer CELL_SHIFT=5,
    parameter integer MIN_H_RUN=3,
    parameter integer MIN_PIXELS=256,
    parameter integer KEEP_MIN_PIXELS=128,
    parameter integer MIN_AREA=256,
    parameter integer MIN_FILL_RATIO=4,
    parameter integer CORE_MIN_PIXELS=16,
    parameter integer WEAK_CELL_MIN_PIXELS=64,
    parameter integer HALO_MIN_PIXELS=4,
    // Acquisition/retention both require real color evidence when enabled.
    // Defaults preserve the earlier permissive detector for legacy callers.
    parameter integer MIN_STRONG_PIXELS=0,
    parameter integer STRONG_FILL_RATIO=0,
    parameter integer WEAK_FALLBACK_ENABLE=1,
    // Optional bounded weak-color recovery. With a positive area limit this
    // pass can qualify without a saturated core, but still needs the normal
    // count, density, aspect and multi-frame confirmation. Legacy default off.
    parameter [2:0] WEAK_FALLBACK_COLOR_MASK=3'b111,
    parameter integer WEAK_FALLBACK_MAX_AREA=0,
    parameter integer MAX_ASPECT_RATIO=0,
    // Keep weak pixels for association/confidence, but draw only the exact
    // strong-pixel extent of connected core cells. Legacy callers default off.
    parameter integer TIGHT_BOUNDS=0,
    parameter integer CORE_QUALITY_ENABLE=0,
    parameter integer MAX_PROCESS_CYCLES=45000,
    parameter integer EDGE_ACQUIRE_MARGIN=0
)(
    input wire I_clk,I_rst_n,I_frame_start,I_line_end,I_de,
    input wire [11:0] I_x,I_y,
    input wire [2:0] I_hit,I_strong_hit,
    input wire I_enable,
    input wire [2:0] I_locked,
    output reg O_result,
    output reg [1:0] O_color,
    output reg O_match,
    // Acquisition confidence from the SAME full component as qualification.
    output reg O_acquire,
    output reg [11:0] O_center_x,O_center_y,
    output reg [11:0] O_left,O_right,O_top,O_bottom,
    output reg [SUM_WIDTH-1:0] O_pixel_count
);
    localparam integer CELL_SIZE=1<<CELL_SHIFT;
    localparam integer GRID_COLS=(IMG_WIDTH+CELL_SIZE-1)/CELL_SIZE;
    localparam integer GRID_ROWS=(IMG_HEIGHT+CELL_SIZE-1)/CELL_SIZE;
    localparam integer CELLS=GRID_COLS*GRID_ROWS;
    localparam integer ADDR_BITS=CELLS>1 ? $clog2(CELLS) : 1;
    localparam integer GX_BITS=GRID_COLS>1 ? $clog2(GRID_COLS) : 1;
    localparam integer GY_BITS=GRID_ROWS>1 ? $clog2(GRID_ROWS) : 1;
    // Colors run sequentially. Frame-wide component IDs allow their halo
    // visit tags to share one field instead of retaining three RAM fields.
    localparam integer TAG_BITS=CELLS>1 ? $clog2(3*CELLS+1) : 2;
    localparam integer CNT_BITS=2*CELL_SHIFT+1;
    localparam integer BASE_FIELD_BITS=2*CNT_BITS+4*CELL_SHIFT;
    localparam integer FIELD_BITS=BASE_FIELD_BITS+(TIGHT_BOUNDS ? 4*CELL_SHIFT : 0);
    localparam integer STAT_BITS=3*FIELD_BITS;
    localparam integer CORE_SEEN=STAT_BITS;
    localparam integer WEAK_SEEN=STAT_BITS+3;
    localparam integer RING_TAG=STAT_BITS+6;
    localparam integer RAM_BITS=STAT_BITS+6+TAG_BITS;
    localparam integer QUEUE_BITS=GX_BITS+GY_BITS;
    localparam integer QUEUE_DEPTH=1<<ADDR_BITS;
    localparam integer RUN_BITS=MIN_H_RUN>1 ? $clog2(MIN_H_RUN+1) : 1;
    // Counts cannot exceed the number of image pixels. Coordinate sums from
    // the legacy centroid implementation must not widen these BFS counters.
    localparam integer COUNT_BITS=$clog2(IMG_WIDTH*IMG_HEIGHT+1);
    localparam integer CYCLE_BITS=MAX_PROCESS_CYCLES>1 ? $clog2(MAX_PROCESS_CYCLES+1) : 1;
    localparam [3:0] CAPTURE=0,SCAN_REQUEST=1,SCAN_RESPONSE=2,
        POP_REQUEST=3,POP_RESPONSE=4,NEIGHBORS=5,DRAIN=6,NEXT_VERTEX=7,
        QUAL_AREA=8,QUAL_RESULT=9,PUBLISH=10,EXPIRE=11;

    reg frame_active;
    reg [RUN_BITS-1:0] run_length[0:2];
    reg [RUN_BITS-1:0] next_run[0:2];
    reg [2:0] prefix_now;
    wire input_active=I_enable && (frame_active || I_frame_start);
    integer rc;
    always @* begin
        prefix_now=0;
        for(rc=0;rc<3;rc=rc+1)begin
            next_run[rc]=0;
            if(input_active && I_de && I_hit[rc])begin
                if(I_frame_start || I_x==0) next_run[rc]=1;
                else if(run_length[rc]<MIN_H_RUN) next_run[rc]=run_length[rc]+1'b1;
                else next_run[rc]=run_length[rc];
                prefix_now[rc]=next_run[rc]>=MIN_H_RUN;
            end
        end
    end
    reg q0_de,q1_de,q0_end,q1_end;
    reg [11:0] q0_x,q0_y,q1_x,q1_y;
    reg [2:0] q0_hit,q1_hit,q0_strong,q1_strong,q0_prefix,q1_prefix;
    wire q0_same=q0_de && q0_y==q1_y;
    wire now_same=input_active && I_de && I_y==q1_y && !I_frame_start;
    wire [2:0] qualified_hit=q1_hit & (q1_prefix |
        ({3{q0_same && MIN_H_RUN>=2}} & q0_prefix) |
        ({3{now_same && MIN_H_RUN>=3}} & prefix_now));
    reg cap_de,cap_end;
    reg [11:0] cap_x,cap_y;
    reg [2:0] cap_hit,cap_strong;
    integer rr;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            frame_active<=0;q0_de<=0;q1_de<=0;cap_de<=0;
            q0_end<=0;q1_end<=0;cap_end<=0;
            q0_x<=0;q0_y<=0;q1_x<=0;q1_y<=0;cap_x<=0;cap_y<=0;
            q0_hit<=0;q1_hit<=0;q0_strong<=0;q1_strong<=0;
            q0_prefix<=0;q1_prefix<=0;cap_hit<=0;cap_strong<=0;
            for(rr=0;rr<3;rr=rr+1)run_length[rr]<=0;
        end else if(!I_enable)begin
            frame_active<=0;q0_de<=0;q1_de<=0;cap_de<=0;
            q0_prefix<=0;q1_prefix<=0;cap_hit<=0;cap_strong<=0;
            for(rr=0;rr<3;rr=rr+1)run_length[rr]<=0;
        end else begin
            if(I_frame_start)frame_active<=1;
            q0_de<=input_active && I_de;q0_end<=I_line_end;
            q0_x<=I_x;q0_y<=I_y;q0_hit<=I_hit;q0_strong<=I_strong_hit;
            q0_prefix<=prefix_now;
            for(rr=0;rr<3;rr=rr+1)run_length[rr]<=I_line_end ? 0 : next_run[rr];
            if(I_frame_start)begin
                q1_de<=0;cap_de<=0;q1_prefix<=0;cap_hit<=0;cap_strong<=0;
            end else begin
                q1_de<=q0_de;q1_end<=q0_end;q1_x<=q0_x;q1_y<=q0_y;
                q1_hit<=q0_hit;q1_strong<=q0_strong;q1_prefix<=q0_prefix;
                cap_de<=q1_de;cap_end<=q1_end;cap_x<=q1_x;cap_y<=q1_y;
                cap_hit<=qualified_hit;cap_strong<=qualified_hit & q1_strong;
            end
        end
    end

    // One synchronous read and one write port. Capture and backend are exclusive.
    reg [RAM_BITS-1:0] cell_ram[0:CELLS-1];
    reg [RAM_BITS-1:0] cell_read_data;
    reg [ADDR_BITS-1:0] cell_read_address,cell_write_address;
    reg cell_write_enable;
    reg [RAM_BITS-1:0] cell_write_data;
    always @(posedge I_clk)begin
        cell_read_data<=cell_ram[cell_read_address];
        if(cell_write_enable)cell_ram[cell_write_address]<=cell_write_data;
    end
    reg [QUEUE_BITS-1:0] queue_ram[0:QUEUE_DEPTH-1];
    reg [QUEUE_BITS-1:0] queue_read_data,queue_write_data;
    reg queue_write_enable;
    reg [ADDR_BITS-1:0] queue_write_address;
    reg [ADDR_BITS:0] queue_head,queue_tail;
    always @(posedge I_clk)begin
        queue_read_data<=queue_ram[queue_head[ADDR_BITS-1:0]];
        if(queue_write_enable)queue_ram[queue_write_address]<=queue_write_data;
    end

    reg [STAT_BITS-1:0] cached_statistics,next_statistics,base_statistics;
    reg [CNT_BITS-1:0] stat_count,stat_strong;
    reg [CELL_SHIFT-1:0] stat_min_x,stat_max_x,stat_min_y,stat_max_y;
    reg [CELL_SHIFT-1:0] strong_min_x,strong_max_x,strong_min_y,strong_max_y;
    integer sc;
    always @* begin
        base_statistics=cached_statistics;
        if(cap_x[CELL_SHIFT-1:0]==0)begin
            if(cap_y[CELL_SHIFT-1:0]==0)base_statistics=0;
            else base_statistics=cell_read_data[STAT_BITS-1:0];
        end
        next_statistics=base_statistics;
        stat_count=0;stat_strong=0;stat_min_x=0;stat_max_x=0;stat_min_y=0;stat_max_y=0;
        strong_min_x=0;strong_max_x=0;strong_min_y=0;strong_max_y=0;
        for(sc=0;sc<3;sc=sc+1)begin
            stat_count=base_statistics[sc*FIELD_BITS +: CNT_BITS];
            stat_strong=base_statistics[sc*FIELD_BITS+CNT_BITS +: CNT_BITS];
            stat_min_x=base_statistics[sc*FIELD_BITS+2*CNT_BITS +: CELL_SHIFT];
            stat_max_x=base_statistics[sc*FIELD_BITS+2*CNT_BITS+CELL_SHIFT +: CELL_SHIFT];
            stat_min_y=base_statistics[sc*FIELD_BITS+2*CNT_BITS+2*CELL_SHIFT +: CELL_SHIFT];
            stat_max_y=base_statistics[sc*FIELD_BITS+2*CNT_BITS+3*CELL_SHIFT +: CELL_SHIFT];
            // A constant-sized shift avoids out-of-range part selects when
            // the optional fields are absent in the legacy RAM layout.
            {strong_max_y,strong_min_y,strong_max_x,strong_min_x}=
                base_statistics[sc*FIELD_BITS +: FIELD_BITS] >> BASE_FIELD_BITS;
            if(cap_hit[sc])begin
                if(stat_count==0)begin
                    stat_min_x=cap_x[CELL_SHIFT-1:0];stat_max_x=cap_x[CELL_SHIFT-1:0];
                    stat_min_y=cap_y[CELL_SHIFT-1:0];stat_max_y=cap_y[CELL_SHIFT-1:0];
                end else begin
                    if(cap_x[CELL_SHIFT-1:0]<stat_min_x)stat_min_x=cap_x[CELL_SHIFT-1:0];
                    if(cap_x[CELL_SHIFT-1:0]>stat_max_x)stat_max_x=cap_x[CELL_SHIFT-1:0];
                    if(cap_y[CELL_SHIFT-1:0]<stat_min_y)stat_min_y=cap_y[CELL_SHIFT-1:0];
                    if(cap_y[CELL_SHIFT-1:0]>stat_max_y)stat_max_y=cap_y[CELL_SHIFT-1:0];
                end
                stat_count=stat_count+1'b1;
                if(cap_strong[sc])begin
                    if(stat_strong==0)begin
                        strong_min_x=cap_x[CELL_SHIFT-1:0];strong_max_x=cap_x[CELL_SHIFT-1:0];
                        strong_min_y=cap_y[CELL_SHIFT-1:0];strong_max_y=cap_y[CELL_SHIFT-1:0];
                    end else begin
                        if(cap_x[CELL_SHIFT-1:0]<strong_min_x)strong_min_x=cap_x[CELL_SHIFT-1:0];
                        if(cap_x[CELL_SHIFT-1:0]>strong_max_x)strong_max_x=cap_x[CELL_SHIFT-1:0];
                        if(cap_y[CELL_SHIFT-1:0]<strong_min_y)strong_min_y=cap_y[CELL_SHIFT-1:0];
                        if(cap_y[CELL_SHIFT-1:0]>strong_max_y)strong_max_y=cap_y[CELL_SHIFT-1:0];
                    end
                    stat_strong=stat_strong+1'b1;
                end
            end
            next_statistics[sc*FIELD_BITS +: FIELD_BITS]=TIGHT_BOUNDS ?
                {strong_max_y,strong_min_y,strong_max_x,strong_min_x,
                 stat_max_y,stat_min_y,stat_max_x,stat_min_x,stat_strong,stat_count} :
                {stat_max_y,stat_min_y,stat_max_x,stat_min_x,stat_strong,stat_count};
        end
    end

    reg [3:0] state;
    reg [1:0] color_index;
    reg weak_pass;
    reg [2:0] locked_snapshot;
    reg [CYCLE_BITS-1:0] backend_cycles;
    reg [ADDR_BITS-1:0] scan_index;
    reg [GX_BITS-1:0] scan_x,vertex_x,neighbor_x,response_x;
    reg [GY_BITS-1:0] scan_y,vertex_y,neighbor_y,response_y;
    reg [1:0] neighbor_number;
    reg neighbor_valid,response_valid;
    reg [ADDR_BITS-1:0] neighbor_address,response_address;
    reg [TAG_BITS-1:0] component_tag;
    reg [COUNT_BITS-1:0] component_count,component_strong,best_count,best_strong;
    reg [11:0] component_left,component_right,component_top,component_bottom;
    reg [11:0] core_left,core_right,core_top,core_bottom;
    reg core_valid;
    reg [11:0] best_left,best_right,best_top,best_bottom;
    reg [25:0] component_area;
    reg best_valid,best_acquire;
    wire [FIELD_BITS-1:0] selected_field=cell_read_data[color_index*FIELD_BITS +: FIELD_BITS];
    wire [CNT_BITS-1:0] selected_count=selected_field[0 +: CNT_BITS];
    wire [CNT_BITS-1:0] selected_strong=selected_field[CNT_BITS +: CNT_BITS];
    wire [CELL_SHIFT-1:0] selected_min_x=selected_field[2*CNT_BITS +: CELL_SHIFT];
    wire [CELL_SHIFT-1:0] selected_max_x=selected_field[2*CNT_BITS+CELL_SHIFT +: CELL_SHIFT];
    wire [CELL_SHIFT-1:0] selected_min_y=selected_field[2*CNT_BITS+2*CELL_SHIFT +: CELL_SHIFT];
    wire [CELL_SHIFT-1:0] selected_max_y=selected_field[2*CNT_BITS+3*CELL_SHIFT +: CELL_SHIFT];
    wire [4*CELL_SHIFT-1:0] selected_core_field=selected_field >> BASE_FIELD_BITS;
    wire [CELL_SHIFT-1:0] core_min_x=selected_core_field[0 +: CELL_SHIFT];
    wire [CELL_SHIFT-1:0] core_max_x=selected_core_field[CELL_SHIFT +: CELL_SHIFT];
    wire [CELL_SHIFT-1:0] core_min_y=selected_core_field[2*CELL_SHIFT +: CELL_SHIFT];
    wire [CELL_SHIFT-1:0] core_max_y=selected_core_field[3*CELL_SHIFT +: CELL_SHIFT];
    wire seen=weak_pass ? cell_read_data[WEAK_SEEN+color_index] : cell_read_data[CORE_SEEN+color_index];
    wire seed_eligible=!seen && (weak_pass ? selected_count>=WEAK_CELL_MIN_PIXELS : selected_strong>=CORE_MIN_PIXELS);
    wire response_core=seed_eligible;
    wire response_halo=!weak_pass && selected_strong<CORE_MIN_PIXELS && selected_count>=HALO_MIN_PIXELS &&
        cell_read_data[RING_TAG +: TAG_BITS]!=component_tag;
    wire process_response=(state==NEIGHBORS || state==DRAIN) && response_valid;
    wire add_neighbor=process_response && (response_core || response_halo);
    wire enqueue_neighbor=process_response && response_core;
    wire [11:0] seed_origin_x=scan_x<<CELL_SHIFT;
    wire [11:0] seed_origin_y=scan_y<<CELL_SHIFT;
    wire [11:0] response_origin_x=response_x<<CELL_SHIFT;
    wire [11:0] response_origin_y=response_y<<CELL_SHIFT;
    wire [11:0] selected_left=response_origin_x+selected_min_x;
    wire [11:0] selected_right=response_origin_x+selected_max_x;
    wire [11:0] selected_top=response_origin_y+selected_min_y;
    wire [11:0] selected_bottom=response_origin_y+selected_max_y;
    wire [11:0] selected_core_left=response_origin_x+core_min_x;
    wire [11:0] selected_core_right=response_origin_x+core_max_x;
    wire [11:0] selected_core_top=response_origin_y+core_min_y;
    wire [11:0] selected_core_bottom=response_origin_y+core_max_y;
    // A weak surrounding cast may fill neighboring halo cells. Tight targets
    // are judged against their actual saturated-core box when opted in.
    wire use_core_quality=CORE_QUALITY_ENABLE && TIGHT_BOUNDS && !weak_pass && core_valid;
    wire [11:0] quality_left=use_core_quality ? core_left:component_left;
    wire [11:0] quality_right=use_core_quality ? core_right:component_right;
    wire [11:0] quality_top=use_core_quality ? core_top:component_top;
    wire [11:0] quality_bottom=use_core_quality ? core_bottom:component_bottom;
    wire [12:0] component_width={1'b0,quality_right}-{1'b0,quality_left}+13'd1;
    wire [12:0] component_height={1'b0,quality_bottom}-{1'b0,quality_top}+13'd1;
    wire [COUNT_BITS+7:0] filled_limit=component_count*MIN_FILL_RATIO;
    wire [COUNT_BITS+7:0] strong_limit=component_strong*STRONG_FILL_RATIO;
    // A peripheral object can lose weak evidence under lens shading even
    // while fully visible. Retain the same saturated-core/shape requirements,
    // and use the already-established retention count in the peripheral band.
    wire peripheral=EDGE_ACQUIRE_MARGIN>0 &&
        (quality_left<EDGE_ACQUIRE_MARGIN || quality_top<EDGE_ACQUIRE_MARGIN ||
         quality_right>=IMG_WIDTH-EDGE_ACQUIRE_MARGIN || quality_bottom>=IMG_HEIGHT-EDGE_ACQUIRE_MARGIN);
    wire bounded_weak=weak_pass && WEAK_FALLBACK_MAX_AREA>0;
    wire shape_ok=MAX_ASPECT_RATIO==0 ||
        (component_width<=component_height*MAX_ASPECT_RATIO &&
         component_height<=component_width*MAX_ASPECT_RATIO);
    wire strong_ok=component_strong>=MIN_STRONG_PIXELS &&
        (STRONG_FILL_RATIO==0 || strong_limit>=(use_core_quality ? component_area:component_count));
    wire qualified=component_count>0 && component_count >= ((locked_snapshot[color_index] || peripheral) ? KEEP_MIN_PIXELS : MIN_PIXELS) &&
        component_area>=MIN_AREA && (MIN_FILL_RATIO==0 || filled_limit>=component_area) &&
        shape_ok && (bounded_weak ? component_area<=WEAK_FALLBACK_MAX_AREA : strong_ok);
    wire better= !best_valid || component_count>best_count ||
        (component_count==best_count && component_strong>best_strong);
    wire [12:0] best_center_x={1'b0,best_left}+{1'b0,best_right};
    wire [12:0] best_center_y={1'b0,best_top}+{1'b0,best_bottom};
    wire capture_write=state==CAPTURE && cap_de && (cap_end || cap_x[CELL_SHIFT-1:0]==CELL_SIZE-1);
    wire capture_eof=state==CAPTURE && cap_de && cap_x==IMG_WIDTH-1 && cap_y==IMG_HEIGHT-1;
    wire backend_active=state!=CAPTURE;

    // Four adjacent read requests issue in consecutive clocks. Their returned
    // records are immediately marked, counted and optionally enqueued.
    always @* begin
        neighbor_x=vertex_x;neighbor_y=vertex_y;neighbor_valid=0;
        case(neighbor_number)
            0:begin neighbor_x=vertex_x-1'b1;neighbor_valid=vertex_x>0;end
            1:begin neighbor_x=vertex_x+1'b1;neighbor_valid=vertex_x<GRID_COLS-1;end
            2:begin neighbor_y=vertex_y-1'b1;neighbor_valid=vertex_y>0;end
            3:begin neighbor_y=vertex_y+1'b1;neighbor_valid=vertex_y<GRID_ROWS-1;end
        endcase
        neighbor_address=neighbor_y*GRID_COLS+neighbor_x;
        cell_read_address=0;
        if(state==CAPTURE && q1_de)cell_read_address=(q1_y>>CELL_SHIFT)*GRID_COLS+(q1_x>>CELL_SHIFT);
        else if(state==SCAN_REQUEST || state==SCAN_RESPONSE)cell_read_address=scan_index;
        else if(state==NEIGHBORS && neighbor_valid)cell_read_address=neighbor_address;
        cell_write_enable=0;cell_write_address=0;cell_write_data=cell_read_data;
        queue_write_enable=0;queue_write_address=queue_tail[ADDR_BITS-1:0];queue_write_data={response_y,response_x};
        if(I_enable && !I_frame_start)begin
            if(capture_write)begin
                cell_write_enable=1;
                cell_write_address=(cap_y>>CELL_SHIFT)*GRID_COLS+(cap_x>>CELL_SHIFT);
                cell_write_data={{(RAM_BITS-STAT_BITS){1'b0}},next_statistics};
            end else if(state==SCAN_RESPONSE && seed_eligible)begin
                cell_write_enable=1;cell_write_address=scan_index;
                if(weak_pass)cell_write_data[WEAK_SEEN+color_index]=1;
                else cell_write_data[CORE_SEEN+color_index]=1;
                queue_write_enable=1;queue_write_address=0;queue_write_data={scan_y,scan_x};
            end else if(add_neighbor)begin
                cell_write_enable=1;cell_write_address=response_address;
                if(response_core)begin
                    if(weak_pass)cell_write_data[WEAK_SEEN+color_index]=1;
                    else cell_write_data[CORE_SEEN+color_index]=1;
                    queue_write_enable=1;
                end else cell_write_data[RING_TAG +: TAG_BITS]=component_tag;
            end
        end
    end

    // Return tags align to the synchronous cell RAM output.
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin response_valid<=0;response_x<=0;response_y<=0;response_address<=0;end
        else if(I_frame_start || !I_enable)response_valid<=0;
        else begin
            response_valid<=state==NEIGHBORS && neighbor_valid;
            response_x<=neighbor_x;response_y<=neighbor_y;response_address<=neighbor_address;
        end
    end

    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            cached_statistics<=0;state<=CAPTURE;color_index<=0;weak_pass<=0;
            locked_snapshot<=0;backend_cycles<=0;scan_index<=0;scan_x<=0;scan_y<=0;
            vertex_x<=0;vertex_y<=0;neighbor_number<=0;queue_head<=0;queue_tail<=0;component_tag<=0;
            component_count<=0;component_strong<=0;component_left<=0;component_right<=0;component_top<=0;component_bottom<=0;
            core_left<=0;core_right<=0;core_top<=0;core_bottom<=0;
            core_valid<=0;
            component_area<=0;best_valid<=0;best_acquire<=0;best_count<=0;best_strong<=0;
            best_left<=0;best_right<=0;best_top<=0;best_bottom<=0;
            O_result<=0;O_color<=0;O_match<=0;O_acquire<=0;O_center_x<=0;O_center_y<=0;
            O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;O_pixel_count<=0;
        end else if(I_frame_start || !I_enable)begin
            cached_statistics<=0;state<=CAPTURE;color_index<=0;weak_pass<=0;
            backend_cycles<=0;scan_index<=0;scan_x<=0;scan_y<=0;
            queue_head<=0;queue_tail<=0;component_tag<=0;best_valid<=0;best_acquire<=0;
            O_result<=0;O_match<=0;O_acquire<=0;O_pixel_count<=0;
        end else begin
            O_result<=0;
            if(state==CAPTURE)begin
                if(cap_de)cached_statistics<=next_statistics;
                if(capture_eof)begin
                    locked_snapshot<=I_locked;backend_cycles<=0;color_index<=0;weak_pass<=0;
                    scan_index<=0;scan_x<=0;scan_y<=0;component_tag<=0;
                    best_valid<=0;best_acquire<=0;best_count<=0;best_strong<=0;
                    state<=SCAN_REQUEST;
                end
            end else begin
                if(state!=EXPIRE)backend_cycles<=backend_cycles+1'b1;
                if(state!=EXPIRE && backend_cycles>=MAX_PROCESS_CYCLES-1)begin
                    state<=EXPIRE;
                end else case(state)
                    SCAN_REQUEST:state<=SCAN_RESPONSE;
                    SCAN_RESPONSE:begin
                        if(seed_eligible)begin
                            if(!weak_pass)component_tag<=component_tag+1'b1;
                            component_count<=selected_count;component_strong<=selected_strong;
                            component_left<=seed_origin_x+selected_min_x;component_right<=seed_origin_x+selected_max_x;
                            component_top<=seed_origin_y+selected_min_y;component_bottom<=seed_origin_y+selected_max_y;
                            core_left<=seed_origin_x+core_min_x;core_right<=seed_origin_x+core_max_x;
                            core_top<=seed_origin_y+core_min_y;core_bottom<=seed_origin_y+core_max_y;
                            core_valid<=selected_strong!=0;
                            queue_head<=0;queue_tail<=1;state<=POP_REQUEST;
                        end else if(scan_index==CELLS-1)begin
                            if(WEAK_FALLBACK_ENABLE && WEAK_FALLBACK_COLOR_MASK[color_index] && !weak_pass && !best_valid)begin
                                weak_pass<=1;scan_index<=0;scan_x<=0;scan_y<=0;
                                state<=SCAN_REQUEST;
                            end else state<=PUBLISH;
                        end else begin
                            scan_index<=scan_index+1'b1;
                            if(scan_x==GRID_COLS-1)begin scan_x<=0;scan_y<=scan_y+1'b1;end
                            else scan_x<=scan_x+1'b1;
                            state<=SCAN_REQUEST;
                        end
                    end
                    POP_REQUEST:begin queue_head<=queue_head+1'b1;state<=POP_RESPONSE;end
                    POP_RESPONSE:begin
                        vertex_x<=queue_read_data[0 +: GX_BITS];vertex_y<=queue_read_data[GX_BITS +: GY_BITS];
                        neighbor_number<=0;state<=NEIGHBORS;
                    end
                    NEIGHBORS:begin
                        if(neighbor_number==3)state<=DRAIN;
                        else neighbor_number<=neighbor_number+1'b1;
                    end
                    DRAIN:state<=NEXT_VERTEX;
                    NEXT_VERTEX:begin
                        if(queue_head<queue_tail)state<=POP_REQUEST;
                        else state<=QUAL_AREA;
                    end
                    QUAL_AREA:begin component_area<=component_width*component_height;state<=QUAL_RESULT;end
                    QUAL_RESULT:begin
                        if(qualified && better)begin
                            best_valid<=1;best_count<=component_count;best_strong<=component_strong;
                            best_acquire<=component_count >= (peripheral ? KEEP_MIN_PIXELS : MIN_PIXELS);
                            // Qualification/ranking still use the complete color
                            // component. Weak fallback retains its original box.
                            best_left<=TIGHT_BOUNDS && !weak_pass && core_valid ? core_left : component_left;
                            best_right<=TIGHT_BOUNDS && !weak_pass && core_valid ? core_right : component_right;
                            best_top<=TIGHT_BOUNDS && !weak_pass && core_valid ? core_top : component_top;
                            best_bottom<=TIGHT_BOUNDS && !weak_pass && core_valid ? core_bottom : component_bottom;
                        end
                        // Resume on the seed cell; its visited flag ensures the
                        // final candidate is committed before end-of-pass choice.
                        state<=SCAN_REQUEST;
                    end
                    PUBLISH:begin
                        O_result<=1;O_color<=color_index+1'b1;O_match<=best_valid;
                        O_acquire<=best_valid && best_acquire;
                        O_pixel_count<=best_valid ? best_count : 0;
                        O_left<=best_valid ? best_left : 0;O_right<=best_valid ? best_right : 0;
                        O_top<=best_valid ? best_top : 0;O_bottom<=best_valid ? best_bottom : 0;
                        O_center_x<=best_valid ? best_center_x[12:1] : 0;
                        O_center_y<=best_valid ? best_center_y[12:1] : 0;
                        if(color_index==2)state<=CAPTURE;
                        else begin
                            color_index<=color_index+1'b1;weak_pass<=0;scan_index<=0;scan_x<=0;scan_y<=0;
                            best_valid<=0;best_acquire<=0;best_count<=0;best_strong<=0;state<=SCAN_REQUEST;
                        end
                    end
                    EXPIRE:begin
                        O_result<=1;O_color<=color_index+1'b1;O_match<=0;O_acquire<=0;O_pixel_count<=0;
                        O_left<=0;O_right<=0;O_top<=0;O_bottom<=0;O_center_x<=0;O_center_y<=0;
                        if(color_index==2)state<=CAPTURE;
                        else color_index<=color_index+1'b1;
                    end
                    default:state<=CAPTURE;
                endcase
                if(add_neighbor)begin
                    component_count<=component_count+selected_count;
                    component_strong<=component_strong+selected_strong;
                    if(selected_left<component_left)component_left<=selected_left;
                    if(selected_right>component_right)component_right<=selected_right;
                    if(selected_top<component_top)component_top<=selected_top;
                    if(selected_bottom>component_bottom)component_bottom<=selected_bottom;
                    // A weak halo may contain isolated strong speckles. Only
                    // qualified, enqueued core cells can enlarge the tight box.
                    if(TIGHT_BOUNDS && response_core && !weak_pass && selected_strong!=0)begin
                        if(!core_valid || selected_core_left<core_left)core_left<=selected_core_left;
                        if(!core_valid || selected_core_right>core_right)core_right<=selected_core_right;
                        if(!core_valid || selected_core_top<core_top)core_top<=selected_core_top;
                        if(!core_valid || selected_core_bottom>core_bottom)core_bottom<=selected_core_bottom;
                        core_valid<=1;
                    end
                    if(enqueue_neighbor)queue_tail<=queue_tail+1'b1;
                end
            end
        end
    end
endmodule
`endif
