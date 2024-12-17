from gnuradio import gr
import numpy as np
import pmt
import sys
from PyQt5 import Qt
import pyqtgraph as pg
from collections import deque
import time


class signal_strength_plot(gr.sync_block):
    def __init__(self, window_size=1000):
        gr.sync_block.__init__(self,
            name="signal_strength_plot",
            in_sig=None,
            out_sig=None)

        self.window_size = window_size
        self.signal_strength = deque(maxlen=window_size)
        self.doppler_shift = deque(maxlen=window_size)
        self.elevation = deque(maxlen=window_size)
        self.time_points = deque(maxlen=window_size)
        self.start_time = None
        
        # GUI setup
        self.widget = Qt.QWidget()
        self.layout = Qt.QVBoxLayout(self.widget)
        
        # Create plots
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.layout.addWidget(self.plot_widget)
        
        # Signal Strength Plot (0-100% range)
        self.strength_plot = self.plot_widget.addPlot(row=0, col=0)
        self.strength_plot.setLabel('left', 'Signal Strength', '%')
        self.strength_plot.setLabel('bottom', 'Time', 's')
        self.strength_plot.showGrid(x=True, y=True)
        self.strength_plot.setYRange(0, 100)
        self.strength_curve = self.strength_plot.plot(pen='y')
        
        # Doppler Shift Plot (±0.1% range)
        self.doppler_plot = self.plot_widget.addPlot(row=1, col=0)
        self.doppler_plot.setLabel('left', 'Doppler Shift', '%')
        self.doppler_plot.setLabel('bottom', 'Time', 's')
        self.doppler_plot.showGrid(x=True, y=True)
        self.doppler_plot.setYRange(-0.1, 0.1)
        self.doppler_curve = self.doppler_plot.plot(pen='g')
        
        # Elevation Plot (0-90 degrees range)
        self.elevation_plot = self.plot_widget.addPlot(row=2, col=0)
        self.elevation_plot.setLabel('left', 'Elevation', 'degrees')
        self.elevation_plot.setLabel('bottom', 'Time', 's')
        self.elevation_plot.showGrid(x=True, y=True)
        self.elevation_plot.setYRange(-90, 90)
        self.elevation_curve = self.elevation_plot.plot(pen='b')
        
        # Message port setup
        self.message_port_register_in(pmt.intern("sat_pos"))
        self.set_msg_handler(pmt.intern("sat_pos"), self.handle_sat_pos)
        
        # Setup update timer
        self.timer = Qt.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(1000)  # (1000ms)
        
    def handle_sat_pos(self, msg):
        try:
            if not pmt.is_dict(msg):
                return
                
            # Get current time using time module
            if self.start_time is None:
                self.start_time = time.time()
            current_time = time.time() - self.start_time
            
            # Extract values from PMT dictionary
            strength = pmt.to_double(pmt.dict_ref(msg, pmt.intern("strength"), pmt.from_double(0.0)))
            doppler = pmt.to_double(pmt.dict_ref(msg, pmt.intern("doppler"), pmt.from_double(1.0)))
            elevation = pmt.to_double(pmt.dict_ref(msg, pmt.intern("elevation"), pmt.from_double(0.0)))
            
            # Store data
            self.time_points.append(current_time)
            self.signal_strength.append(strength * 100.0)  # Convert to percentage
            self.doppler_shift.append((doppler - 1.0) * 100.0)  # Convert to percentage
            self.elevation.append(elevation)
            
            # Debug output
            print(f"Debug - Plot Data: Time={current_time:.1f}s", file=sys.stderr)
            print(f"  Strength={strength*100:.2f}%", file=sys.stderr)
            print(f"  Doppler={((doppler-1.0)*100):.4f}%", file=sys.stderr)
            print(f"  Elevation={elevation:.1f}°", file=sys.stderr)
            
        except Exception as e:
            print(f"Error in handle_sat_pos: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
            
    def update_plots(self):
        try:
            if len(self.time_points) > 0:
                time_array = np.array(self.time_points)
                strength_array = np.array(self.signal_strength)
                doppler_array = np.array(self.doppler_shift)
                elevation_array = np.array(self.elevation)
                
                # Update plots
                self.strength_curve.setData(time_array, strength_array)
                self.doppler_curve.setData(time_array, doppler_array)
                self.elevation_curve.setData(time_array, elevation_array)
                
                if len(time_array) > 1:
                    x_min = max(0, time_array[-1] - 60)
                    x_max = time_array[-1] + 1
                    self.strength_plot.setXRange(x_min, x_max)
                    self.doppler_plot.setXRange(x_min, x_max)
                    self.elevation_plot.setXRange(x_min, x_max)
                
        except Exception as e:
            print(f"Error updating plots: {e}", file=sys.stderr)
            
    def get_widget(self):
        return self.widget