// Volume2: 256 independent 5401-sector slots. PIC2 payload is raw R,G,B bytes.
// Header is a custom 512-byte record, not a directly mountable BMP file. A PC
// exporter must swap RGB->BGR when constructing a conventional 24-bit BMP.
    function [7:0] volume_byte;
        input [8:0] a;
        begin
            case(a)
              0:volume_byte="F";1:volume_byte="P";2:volume_byte="G";3:volume_byte="A";
              4:volume_byte="S";5:volume_byte="D";6:volume_byte="0";7:volume_byte="2";
              8:volume_byte=2;13:volume_byte=8'h08;
              16:volume_byte=8'h19;17:volume_byte=8'h15;
              21:volume_byte=1;25:volume_byte=5;
              28:volume_byte=8'hd0;29:volume_byte=2;
              508:volume_byte="V";509:volume_byte="O";510:volume_byte="L";511:volume_byte="!";
              default:volume_byte=0;
            endcase
        end
    endfunction
// Optional backward-compatible album extension. Unmarked PIC2 records remain
// legacy photos: never infer their source from their physical slot number.
// 98..101="ALB1", 102=source (1 manual / 2 motion), 103=~source, 104=1.
// Every other reserved byte remains zero and must still be validated.
    function [7:0] photo_album_byte;
        input [8:0] a;input [8:0] number;input [1:0] source;
        begin
            photo_album_byte=photo_byte(a,number);
            if(source==1 || source==2)begin
                case(a)
                    98:photo_album_byte="A";99:photo_album_byte="L";
                    100:photo_album_byte="B";101:photo_album_byte="1";
                    102:photo_album_byte={6'b0,source};
                    103:photo_album_byte=~{6'b0,source};
                    104:photo_album_byte=1;
                    default:begin end
                endcase
            end
        end
    endfunction
    function [7:0] photo_byte;
        input [8:0] a;input [8:0] number;
        begin
            case(a)
              0:photo_byte="B";1:photo_byte="M";
              3:photo_byte=8'h32;4:photo_byte=8'h2a;
              11:photo_byte=2;14:photo_byte=40;19:photo_byte=5;
              22:photo_byte=8'h30;23:photo_byte=8'hfd;24,25:photo_byte=8'hff;
              26:photo_byte=1;28:photo_byte=24;
              35:photo_byte=8'h30;36:photo_byte=8'h2a;
              80:photo_byte="F";81:photo_byte="P";82:photo_byte="G";83:photo_byte="A";
              84:photo_byte="P";85:photo_byte="I";86:photo_byte="C";87:photo_byte="2";
              88:photo_byte="D";89:photo_byte="O";90:photo_byte="N";91:photo_byte="E";
              92:photo_byte=number[7:0];93:photo_byte={7'b0,number[8]};
              94:photo_byte="R";95:photo_byte="G";96:photo_byte="B";97:photo_byte=1;
              508:photo_byte="E";509:photo_byte="N";510:photo_byte="D";511:photo_byte="!";
              default:photo_byte=0;
            endcase
        end
    endfunction
