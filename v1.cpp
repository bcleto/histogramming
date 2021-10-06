#include <stdint.h>
#include "hls_stream.h"
#include "ap_int.h"
#include "ap_utils.h"

const int DATA_MAX_COUNT_BIT = 8;
const int TIMEBIN_BIT = 6;
//const long RAM_DEPTH = 2048 * (2**TIMEBIN_BIT);

void part_hist_v1(
		ap_uint<115> data_in,
		ap_uint<1>   input_valid,
		ap_uint<32>  accumulation,
		hls::stream<ap_uint<16> >  &frequency_out,
		hls::stream<ap_uint<16> >  &mode_out

		)
{
//#pragma HLS INTERFACE ap_vld port=data_in
//#pragma HLS INTERFACE ap_fifo port=data_out_fifo
//#pragma HLS INTERFACE ap_fifo port=pixel_addr_fifo


	int loop_number = 1;
ap_uint<115> complete_data;
ap_uint<14> tdc_value;
ap_uint<6> tdc_value_compressed;
ap_uint<3> mini_module_addr[8];
ap_uint<7> pixel_true_addr[8];
ap_uint<4> submodule_addr;
ap_uint<11> pixel_true_w_mm[8];
ap_uint<7> pixel_memory_location[8];

ap_uint<14> memory[8192]; // 128* (2**TIMEBIN_BIT)
ap_uint<11> memory_addr;

ap_uint<8> memory_value_read, memory_value_write ;

ap_uint<DATA_MAX_COUNT_BIT> data_count_value;
ap_uint<TIMEBIN_BIT> tdc_read_value;


ap_uint<16> mode[128];
ap_uint<16> frequency[128];

for(unsigned long r=0;r<8; r++){	//2048
	pixel_true_w_mm[r]=0;
	pixel_memory_location[r]=0;
}


for(unsigned long r=0;r<128; r++){	//2048
	mode[r]=0;
	frequency[r]=0;
}


for(unsigned long r=0;r<8192; r++){	//works with prereset enabled
	memory[r]=0;
}


for(long t=0;t<=accumulation;t++){
	ap_wait_until(input_valid);	// wait for trigger

		complete_data=data_in;
		tdc_value = complete_data.range(109, 96);
		tdc_value_compressed = tdc_value.range(TIMEBIN_BIT-1,0);
		submodule_addr = complete_data.range(114, 111);

		mini_module_addr[0] = complete_data.range(8,6);
		mini_module_addr[1] = complete_data.range(20,18);
		mini_module_addr[2] = complete_data.range(32,30);
		mini_module_addr[3] = complete_data.range(44,42);
		mini_module_addr[4] = complete_data.range(56,54);
		mini_module_addr[5] = complete_data.range(68,66);
		mini_module_addr[6] = complete_data.range(80,78);
		mini_module_addr[7] = complete_data.range(92,90);


		data_partition: for (int p = 0; p < 8; p++) {
#pragma HLS UNROLL
			//mini_module_addr[p] = complete_data.range(12 * p + 8,
				//	12 * p + 6);
			pixel_true_addr[p] = (mini_module_addr[p] * 16);
			pixel_true_w_mm[p] = (ap_uint<11> (submodule_addr) << 7)
					+ pixel_true_addr[p];

			pixel_memory_location[p] = pixel_true_w_mm[p]/16;
			//pixel_memory_location[p] = pixel_true_w_mm[p]>>4;

			memory_addr = (ap_uint<17> (tdc_value_compressed)<<11) + (pixel_memory_location [p]);
			memory_value_read = memory[memory_addr]; 	//data count
			memory_value_write = memory_value_read+1;
			memory[memory_addr] = memory_value_write;

			if(frequency[pixel_memory_location[p]]<=memory_value_write)
			{
				frequency[pixel_memory_location[p]]=memory_value_write;
				mode[pixel_memory_location[p]]=tdc_value_compressed;
			}
			else{
			}

		}

}


				for(unsigned int r=0;r<128; r++){	//flush out data
		#pragma HLS PIPELINE II=1
					frequency_out.write(frequency[r]);
					mode_out.write(mode[r]);
				}
}
