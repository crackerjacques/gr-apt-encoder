from gnuradio import gr
import numpy as np
import pmt
from PyQt5 import Qt
from PyQt5.QtSvg import QSvgRenderer
import sys
from gnuradio import qtgui
from gnuradio.apt_encoder import EARTH_RADIUS, LIGHT_SPEED, NOAA_ALTITUDE, PI, DEG_TO_RAD, RAD_TO_DEG

class position_visualizer(gr.sync_block):
    """
    Qt-based satellite position visualizer with SVG map and zoom capabilities
    """
    def __init__(self, num_sats=1, sat_names=None):
        gr.sync_block.__init__(self,
            name="position_visualizer",
            in_sig=None,
            out_sig=None)

        print("Initializing position_visualizer", file=sys.stderr)
        
        # Initialize satellite states
        self.num_sats = num_sats
        self.sat_names = sat_names if sat_names else [f"SAT-{i+1}" for i in range(num_sats)]
        
        # Create state dictionary for each satellite
        self.states = {}
        for i in range(num_sats):
            # Generate distinct colors using HSV color space
            hue = (i * 360 / num_sats) % 360
            color = Qt.QColor.fromHsv(int(hue), 255, 255)
            self.states[i] = {
                "name": self.sat_names[i],
                "latitude": 0.0,
                "longitude": 0.0,
                "elevation": 0.0,
                "azimuth": 0.0,
                "strength": 0.0,
                "doppler": 1.0,
                "heading": 0.0,
                "range": 0.0,
                "velocity": 0.0,
                "color": color
            }

        # Ground station state
        self.gs_state = {
            "antennaLat": 35.0,
            "antennaLon": 135.0,
            "antennaAlt": 0.0,
            "antennaAzimuth": 0.0,
            "antennaElevation": 0.0,
        }

        # Register message ports for each satellite
        for i in range(num_sats):
            port_name = f"sat_pos{i}" if i > 0 else "sat_pos"
            self.message_port_register_in(pmt.intern(port_name))
            self.set_msg_handler(pmt.intern(port_name), lambda msg, idx=i: self.handle_sat_pos(msg, idx))
        
        # GUI setup
        self.widget = Qt.QWidget()
        self.layout = Qt.QVBoxLayout(self.widget)
        
        # Create info panel
        self.info_frame = Qt.QFrame(self.widget)
        self.info_frame.setFrameStyle(Qt.QFrame.StyledPanel)
        self.info_layout = Qt.QVBoxLayout(self.info_frame)
        
        # Add title
        self.title_label = Qt.QLabel("Multi-Satellite Position Monitor", self.info_frame)
        self.title_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.info_layout.addWidget(self.title_label)

        # Create satellite info frames
        self.sat_frames = {}
        for i in range(num_sats):
            sat_frame = Qt.QFrame(self.info_frame)
            sat_frame.setFrameStyle(Qt.QFrame.StyledPanel)
            sat_layout = Qt.QVBoxLayout(sat_frame)
            
            # Set background color with low alpha
            color = self.states[i]['color']
            sat_frame.setStyleSheet(f"background-color: rgba({color.red()}, {color.green()}, {color.blue()}, 30);")
            
            sat_title = Qt.QLabel(f"{self.sat_names[i]} Status", sat_frame)
            sat_title.setStyleSheet("font-weight: bold;")
            sat_layout.addWidget(sat_title)
            
            self.sat_frames[i] = {
                'frame': sat_frame,
                'pos_label': Qt.QLabel("Position: Waiting for data...", sat_frame),
                'elevation_label': Qt.QLabel("Elevation: --", sat_frame),
                'azimuth_label': Qt.QLabel("Azimuth: --", sat_frame),
                'doppler_label': Qt.QLabel("Doppler: --", sat_frame),
                'range_label': Qt.QLabel("Range: --", sat_frame),
                'strength_label': Qt.QLabel("Signal: --", sat_frame)
            }
            
            for label in self.sat_frames[i].values():
                if isinstance(label, Qt.QLabel):
                    sat_layout.addWidget(label)
            
            self.info_layout.addWidget(sat_frame)
        
        # Ground station info
        self.gs_frame = Qt.QFrame(self.info_frame)
        self.gs_frame.setFrameStyle(Qt.QFrame.StyledPanel)
        self.gs_layout = Qt.QVBoxLayout(self.gs_frame)
        
        self.gs_title = Qt.QLabel("Ground Station", self.gs_frame)
        self.gs_title.setStyleSheet("font-weight: bold;")
        self.gs_layout.addWidget(self.gs_title)
        
        self.gs_pos_label = Qt.QLabel("Position: Waiting for data...", self.gs_frame)
        self.gs_alt_label = Qt.QLabel("Altitude: --", self.gs_frame)
        self.gs_az_label = Qt.QLabel("Azimuth: --", self.gs_frame)
        self.gs_el_label = Qt.QLabel("Elevation: --", self.gs_frame)
        
        self.gs_layout.addWidget(self.gs_pos_label)
        self.gs_layout.addWidget(self.gs_alt_label)
        self.gs_layout.addWidget(self.gs_az_label)
        self.gs_layout.addWidget(self.gs_el_label)
        
        self.info_layout.addWidget(self.gs_frame)
        
        # Status indicator
        self.status_label = Qt.QLabel("Status: Initializing", self.info_frame)
        self.status_label.setStyleSheet("color: blue;")
        self.info_layout.addWidget(self.status_label)
        
        # Add info frame to main layout
        self.layout.addWidget(self.info_frame)
        
        # Create map frame
        self.grid_frame = Qt.QFrame(self.widget)
        self.grid_frame.setFrameStyle(Qt.QFrame.StyledPanel)
        self.grid_frame.setMinimumHeight(400)
        self.grid_frame.paintEvent = self.paint_grid
        self.layout.addWidget(self.grid_frame)
        
        # SVG renderer will be set from GRC template
        self.svg_renderer = None
        
        # Add zoom and pan state
        self.zoom_level = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.dragging = False
        self.last_mouse_pos = None
        
        # Enable mouse tracking
        self.grid_frame.setMouseTracking(True)
        
        # Override mouse events
        self.grid_frame.wheelEvent = self.handle_wheel
        self.grid_frame.mousePressEvent = self.handle_mouse_press
        self.grid_frame.mouseReleaseEvent = self.handle_mouse_release
        self.grid_frame.mouseMoveEvent = self.handle_mouse_move
        
        self._gui_hint = None
        self._update_pending = False
        
        # Setup update timer
        self.update_timer = Qt.QTimer()
        self.update_timer.timeout.connect(self._process_update)
        self.update_timer.start(16)  # ~60fps

    def handle_wheel(self, event):
        """Handle mouse wheel for zooming"""
        mouse_pos = event.pos()
        zoom_factor = 1.1 if event.angleDelta().y() > 0 else 1/1.1
        
        old_world_x = (mouse_pos.x() - self.pan_x) / self.zoom_level
        old_world_y = (mouse_pos.y() - self.pan_y) / self.zoom_level
        
        self.zoom_level *= zoom_factor
        self.zoom_level = max(0.5, min(10.0, self.zoom_level))
        
        new_world_x = (mouse_pos.x() - self.pan_x) / self.zoom_level
        new_world_y = (mouse_pos.y() - self.pan_y) / self.zoom_level
        
        self.pan_x += (new_world_x - old_world_x) * self.zoom_level
        self.pan_y += (new_world_y - old_world_y) * self.zoom_level
        
        self.grid_frame.update()

    def handle_mouse_press(self, event):
        """Handle mouse press for panning"""
        if event.button() == Qt.Qt.LeftButton:
            self.dragging = True
            self.last_mouse_pos = event.pos()

    def handle_mouse_release(self, event):
        """Handle mouse release"""
        if event.button() == Qt.Qt.LeftButton:
            self.dragging = False

    def handle_mouse_move(self, event):
        """Handle mouse move for panning and coordinate display"""
        if self.dragging and self.last_mouse_pos:
            delta = event.pos() - self.last_mouse_pos
            self.pan_x += delta.x()
            self.pan_y += delta.y()
            self.last_mouse_pos = event.pos()
            self.grid_frame.update()
        
        width = self.grid_frame.width()
        height = self.grid_frame.height()
        mouse_x = (event.pos().x() - self.pan_x) / self.zoom_level
        mouse_y = (event.pos().y() - self.pan_y) / self.zoom_level
        
        lon = (mouse_x / width * 360.0) - 180.0
        lat = 90.0 - (mouse_y / height * 180.0)
        
        self.status_label.setText(f"Position: {lat:.2f}°N, {lon:.2f}°E")

    def handle_sat_pos(self, msg, sat_idx):
        """Message handler for satellite position updates"""
        try:
            if not pmt.is_dict(msg):
                return
                
            data = {}
            for i in range(pmt.length(msg)):
                key = pmt.car(pmt.nth(i, msg))
                value = pmt.cdr(pmt.nth(i, msg))
                if pmt.is_symbol(key):
                    key_str = pmt.symbol_to_string(key)
                    if pmt.is_number(value):
                        data[key_str] = pmt.to_double(value)

            # Update ground station data if present
            for key in ["antennaLat", "antennaLon", "antennaAlt", 
                       "antennaAzimuth", "antennaElevation"]:
                if key in data:
                    self.gs_state[key] = data[key]

            # Update satellite specific data
            for key in ["latitude", "longitude", "elevation", "azimuth",
                       "strength", "doppler", "heading", "range", "velocity"]:
                if key in data:
                    self.states[sat_idx][key] = data[key]
            
            self._update_pending = True
            
        except Exception as e:
            print(f"Error processing message for satellite {sat_idx}: {e}", file=sys.stderr)

    def _process_update(self):
        """Timer-based GUI update handler"""
        if not self._update_pending:
            return
            
        try:
            self._update_gui()
            self._update_pending = False
        except Exception as e:
            print(f"Error in GUI update: {e}", file=sys.stderr)

    def _update_gui(self):
        """Update all GUI elements with current state"""
        try:
            # Update ground station info
            self.gs_pos_label.setText(
                f"Position: {self.gs_state['antennaLat']:.3f}°N, {self.gs_state['antennaLon']:.3f}°E")
            self.gs_alt_label.setText(
                f"Altitude: {self.gs_state['antennaAlt']:.1f}m")
            self.gs_az_label.setText(
                f"Azimuth: {self.gs_state['antennaAzimuth']:.1f}°")
            self.gs_el_label.setText(
                f"Elevation: {self.gs_state['antennaElevation']:.1f}°")
            
            # Update each satellite's info
            for sat_idx, state in self.states.items():
                frame = self.sat_frames[sat_idx]
                frame['pos_label'].setText(
                    f"Position: {state['latitude']:.3f}°N, {state['longitude']:.3f}°E")
                frame['elevation_label'].setText(
                    f"Elevation: {state['elevation']:.1f}°")
                frame['azimuth_label'].setText(
                    f"Azimuth: {state['azimuth']:.1f}°")
                frame['doppler_label'].setText(
                    f"Doppler: {state['doppler']:.6f}")
                frame['range_label'].setText(
                    f"Range: {state['range']:.1f}km")
                frame['strength_label'].setText(
                    f"Signal: {state['strength']*100:.1f}%")
            
            # Schedule grid update
            self.grid_frame.update()
            
        except Exception as e:
            print(f"Error updating GUI: {e}", file=sys.stderr)

    def paint_grid(self, event):
        """Paint the world map grid with satellite and ground station positions"""
        painter = None
        try:
            painter = Qt.QPainter(self.grid_frame)
            painter.setRenderHint(Qt.QPainter.Antialiasing)
            
            width = self.grid_frame.width()
            height = self.grid_frame.height()
            
            # Clear background
            painter.fillRect(0, 0, width, height, Qt.QColor(240, 240, 240))
            
            # Apply zoom and pan transformation
            painter.translate(self.pan_x, self.pan_y)
            painter.scale(self.zoom_level, self.zoom_level)
            
            scaled_width = width / self.zoom_level
            scaled_height = height / self.zoom_level
            
            # Draw SVG map if available
            if self.svg_renderer and not self.svg_renderer.isValid():
                self.svg_renderer = None
            
            if self.svg_renderer:
                self.svg_renderer.render(painter, Qt.QRectF(0, 0, scaled_width, scaled_height))
            
            # Calculate visible area
            visible_left = max(-180, -180 + ((-self.pan_x / self.zoom_level) / width * 360))
            visible_right = min(180, -180 + ((width - self.pan_x) / self.zoom_level / width * 360))
            visible_top = max(-90, 90 - (((-self.pan_y) / self.zoom_level) / height * 180))
            visible_bottom = min(90, 90 - ((height - self.pan_y) / self.zoom_level / height * 180))
            
            # Draw grid lines
            grid_spacing = max(1, int(5 / self.zoom_level))
            
            painter.setPen(Qt.QPen(Qt.QColor(200, 200, 200, 100)))
            for lon in range(int(visible_left), int(visible_right) + grid_spacing, grid_spacing):
                x = int(((lon + 180) / 360.0) * scaled_width)
                painter.drawLine(x, 0, x, int(scaled_height))
            
            # Draw latitude lines
            for lat in range(int(visible_top), int(visible_bottom) + grid_spacing, grid_spacing):
                y = int(((90 - lat) / 180.0) * scaled_height)
                painter.drawLine(0, y, int(scaled_width), y)
            
            # Draw major grid lines with labels
            painter.setPen(Qt.QPen(Qt.QColor(150, 150, 150)))
            for lon in range(-180, 181, 30):
                x = int(((lon + 180) / 360.0) * scaled_width)
                painter.drawLine(x, 0, x, int(scaled_height))
                if grid_spacing >= 10:
                    painter.drawText(x - 15, int(scaled_height) - 5, f"{lon}°")
            
            for lat in range(-90, 91, 30):
                y = int(((90 - lat) / 180.0) * scaled_height)
                painter.drawLine(0, y, int(scaled_width), y)
                if grid_spacing >= 10:
                    painter.drawText(5, y + 15, f"{lat}°")

            def coord_to_pixels(lat, lon):
                """Convert lat/lon to pixel coordinates"""
                x = int(((lon + 180.0) / 360.0) * scaled_width)
                y = int(((90.0 - lat) / 180.0) * scaled_height)
                return x, y

            def draw_direction_arrow(x, y, azimuth, elevation, color, label_text, is_antenna=False):
                """Draw an arrow indicating direction with label"""
                angle_rad = (90 - azimuth) * np.pi / 180
                
                # Adjust arrow length based on elevation
                arrow_length = 30 * np.cos(elevation * np.pi / 180)
                
                end_x = int(x + arrow_length * np.cos(angle_rad))
                end_y = int(y - arrow_length * np.sin(angle_rad))
                
                painter.setPen(Qt.QPen(color, 2))
                painter.drawLine(x, y, end_x, end_y)
                
                label_y = y + 20 if is_antenna else y - 20
                
                # Draw label background and text
                metrics = Qt.QFontMetrics(painter.font())
                label_lines = label_text.split('\n')
                text_rects = [metrics.boundingRect(line) for line in label_lines]
                total_width = max(rect.width() for rect in text_rects)
                total_height = sum(rect.height() for rect in text_rects)

                background_rect = Qt.QRectF(
                    x - total_width/2 - 5,
                    label_y - total_height/2 - 2,
                    total_width + 10,
                    total_height + 4
                )

                font = painter.font()
                font.setPointSize(8)
                painter.setFont(font)

                painter.setPen(Qt.Qt.NoPen)
                bg_color = Qt.QColor(255, 255, 255, 180)
                painter.setBrush(Qt.QBrush(bg_color))
                painter.drawRoundedRect(background_rect, 3, 3)

                painter.setPen(color)
                current_y = label_y - total_height/2
                for line in label_lines:
                    rect = metrics.boundingRect(line)
                    painter.drawText(
                        int(x - rect.width()/2),
                        int(current_y + rect.height()),
                        line
                    )
                    current_y += rect.height()
                
                # Draw arrow head
                arrow_head_size = 10
                angle1 = angle_rad + np.pi * 0.875
                angle2 = angle_rad - np.pi * 0.875
                
                points = [
                    Qt.QPoint(end_x, end_y),
                    Qt.QPoint(int(end_x - arrow_head_size * np.cos(angle1)),
                             int(end_y + arrow_head_size * np.sin(angle1))),
                    Qt.QPoint(int(end_x - arrow_head_size * np.cos(angle2)),
                             int(end_y + arrow_head_size * np.sin(angle2)))
                ]
                
                painter.setPen(Qt.QPen(color, 1))
                painter.setBrush(Qt.QBrush(color))
                painter.drawPolygon(Qt.QPolygon(points))

            # Draw ground station
            gs_x, gs_y = coord_to_pixels(self.gs_state['antennaLat'], 
                                       self.gs_state['antennaLon'])
            
            # Ground station highlight circle
            painter.setPen(Qt.QPen(Qt.QColor(0, 0, 255, 40)))
            painter.setBrush(Qt.QBrush(Qt.QColor(0, 0, 255, 20)))
            painter.drawEllipse(Qt.QPointF(gs_x, gs_y), 15, 15)
            
            # Draw antenna direction arrow
            draw_direction_arrow(gs_x, gs_y, 
                            self.gs_state['antennaAzimuth'],
                            self.gs_state['antennaElevation'],
                            Qt.QColor(0, 0, 255),
                            f"Ground Station\n"
                            f"{self.gs_state['antennaLat']:.3f}°N, "
                            f"{self.gs_state['antennaLon']:.3f}°E\n"
                            f"El: {self.gs_state['antennaElevation']:.1f}°",
                            True)
            
            # Ground station range circle (for visibility)
            horizon_range = int(np.sqrt(2 * EARTH_RADIUS * 
                              (self.gs_state['antennaAlt'] / 1000.0)))
            if horizon_range > 0:
                painter.setPen(Qt.QPen(Qt.QColor(0, 0, 255, 30)))
                painter.setBrush(Qt.QBrush(Qt.QColor(0, 0, 255, 10)))
                pixel_range = horizon_range * scaled_width / (2 * np.pi * EARTH_RADIUS)
                painter.drawEllipse(Qt.QPointF(gs_x, gs_y), 
                                  pixel_range, 
                                  pixel_range * np.cos(self.gs_state['antennaLat'] * np.pi / 180))
            
            # Ground station marker
            painter.setPen(Qt.QPen(Qt.QColor(0, 0, 255), 2))
            painter.setBrush(Qt.QBrush(Qt.QColor(0, 0, 255, 180)))
            painter.drawEllipse(Qt.QPointF(gs_x, gs_y), 4, 4)
            
            # Draw each satellite
            for sat_idx, state in self.states.items():
                sat_x, sat_y = coord_to_pixels(state['latitude'], state['longitude'])
                color = state['color']
                
                # Satellite highlight circle
                painter.setPen(Qt.QPen(Qt.QColor(color.red(), color.green(), 
                                               color.blue(), 40)))
                painter.setBrush(Qt.QBrush(Qt.QColor(color.red(), color.green(), 
                                                   color.blue(), 20)))
                painter.drawEllipse(Qt.QPointF(sat_x, sat_y), 15, 15)
                
                # Draw satellite direction arrow
                draw_direction_arrow(sat_x, sat_y,
                                state['heading'], 0,
                                color,
                                f"{state['name']}\n"
                                f"El: {state['elevation']:.1f}°\n"
                                f"Az: {state['azimuth']:.1f}°\n"
                                f"Signal: {state['strength']*100:.0f}%")
                
                # Satellite marker
                painter.setPen(Qt.QPen(color, 2))
                painter.setBrush(Qt.QBrush(color))
                painter.drawEllipse(Qt.QPointF(sat_x, sat_y), 4, 4)
            
        except Exception as e:
            print(f"Error in paint event: {e}", file=sys.stderr)
        finally:
            if painter is not None:
                painter.end()

    def set_gui_hint(self, hint):
        """Set the GUI hint for positioning the widget"""
        self._gui_hint = hint
        
    def get_widget(self):
        """Return the Qt widget for display"""
        return self.widget