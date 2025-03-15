# nexys-video-audio-passthrough

## Purpose
The purpose of this repo is to show how to implement audio passthrough from the line-in and line-out jacks on the Nexys Video board from Digilent using the onboard ADAU1761 audio codec. TO assist, I'm using old digilent IP from [this](https://digilent.com/reference/learn/programmable-logic/tutorials/nexys-video-looper-demo/start?redirect=1) example project written in VHDL and updating it to SystemVerilog because nobody likes VHDL.

I recently needed to get audio working on this codec and really struggled to find resources that weren't either for microcontrollers or softcores, the previously linked example project was the only pure HDL implementation I could find after many hours of searching so hopefully this helps anyone trying to interface with the ADAU1761 on an FPGA.
