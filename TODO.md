### Enhancements Needed for PixElv

1. **Check Monitor Refresh Rate (Hz):**
   - Implement a mechanism to check the refresh rate (Hz) of the monitor.

2. **Default Framerate to Monitor Refresh Rate:**
   - If the user does not specify a frame rate, default the frame rate to the monitor's refresh rate.

3. **Dynamic Queue Size Based on Refresh Rate:**
   - Create a logic to generate a queue size that is appropriate for the monitor's refresh rate. 
   - Proposed formula: `queue size = hz/2`

4. **Allow User-Set Queue Size:**
   - Enable users to set their preferred queue size. 
   - This should override the dynamically generated queue size.