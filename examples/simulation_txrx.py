#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Wifi DSSS Simulation
# Author: Teng-Hui Huang
# GNU Radio version: 3.10.11.0

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5.QtCore import QObject, pyqtSlot
from gnuradio import blocks
import pmt
from gnuradio import blocks, gr
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import gr, pdu
from gnuradio import pdu
from gnuradio import wifi_dsss
import threading



class simulation_txrx(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Wifi DSSS Simulation", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Wifi DSSS Simulation")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "simulation_txrx")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.rate = rate = 0
        self.variable_qtgui_label_0 = variable_qtgui_label_0 = "Long Preamble" if rate < 4 else "Short Preamble"
        self.samp_rate = samp_rate = 32000

        ##################################################
        # Blocks
        ##################################################

        # Create the options list
        self._rate_options = [0, 1, 2, 3, 4, 5, 6]
        # Create the labels list
        self._rate_labels = ['LONG1M', 'LONG2M', 'LONG5_5M', 'LONG11M', 'SHORT2M', 'SHORT5_5M', 'SHORT11M']
        # Create the combo box
        self._rate_tool_bar = Qt.QToolBar(self)
        self._rate_tool_bar.addWidget(Qt.QLabel("Rate" + ": "))
        self._rate_combo_box = Qt.QComboBox()
        self._rate_tool_bar.addWidget(self._rate_combo_box)
        for _label in self._rate_labels: self._rate_combo_box.addItem(_label)
        self._rate_callback = lambda i: Qt.QMetaObject.invokeMethod(self._rate_combo_box, "setCurrentIndex", Qt.Q_ARG("int", self._rate_options.index(i)))
        self._rate_callback(self.rate)
        self._rate_combo_box.currentIndexChanged.connect(
            lambda i: self.set_rate(self._rate_options[i]))
        # Create the radio buttons
        self.top_layout.addWidget(self._rate_tool_bar)
        self.wifi_dsss_ppdu_prefixer_0 = wifi_dsss.ppdu_prefixer(rate)
        self.wifi_dsss_ppdu_chip_mapper_bc_0 = wifi_dsss.ppdu_chip_mapper_bc("psdu_len")
        self.wifi_dsss_chip_sync_c_0 = wifi_dsss.chip_sync_c(rate<4, 0.8)
        self._variable_qtgui_label_0_tool_bar = Qt.QToolBar(self)

        if None:
            self._variable_qtgui_label_0_formatter = None
        else:
            self._variable_qtgui_label_0_formatter = lambda x: str(x)

        self._variable_qtgui_label_0_tool_bar.addWidget(Qt.QLabel("RX Expecting Preamble Type: "))
        self._variable_qtgui_label_0_label = Qt.QLabel(str(self._variable_qtgui_label_0_formatter(self.variable_qtgui_label_0)))
        self._variable_qtgui_label_0_tool_bar.addWidget(self._variable_qtgui_label_0_label)
        self.top_layout.addWidget(self._variable_qtgui_label_0_tool_bar)
        self.pdu_random_pdu_0 = pdu.random_pdu(50, 200, 0xFF, 2)
        self.pdu_pdu_to_tagged_stream_0 = pdu.pdu_to_tagged_stream(gr.types.byte_t, 'psdu_len')
        self.blocks_throttle2_0 = blocks.throttle( gr.sizeof_gr_complex*1, samp_rate, True, 0 if "auto" == "auto" else max( int(float(0.1) * samp_rate) if "auto" == "time" else int(0.1), 1) )
        self.blocks_message_strobe_0 = blocks.message_strobe(pmt.intern("TEST"), 1000)
        self.blocks_message_debug_0 = blocks.message_debug(True, gr.log_levels.info)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.blocks_message_strobe_0, 'strobe'), (self.pdu_random_pdu_0, 'generate'))
        self.msg_connect((self.pdu_random_pdu_0, 'pdus'), (self.wifi_dsss_ppdu_prefixer_0, 'psdu_in'))
        self.msg_connect((self.wifi_dsss_chip_sync_c_0, 'psdu_out'), (self.blocks_message_debug_0, 'print'))
        self.msg_connect((self.wifi_dsss_ppdu_prefixer_0, 'ppdu_out'), (self.pdu_pdu_to_tagged_stream_0, 'pdus'))
        self.connect((self.blocks_throttle2_0, 0), (self.wifi_dsss_chip_sync_c_0, 0))
        self.connect((self.pdu_pdu_to_tagged_stream_0, 0), (self.wifi_dsss_ppdu_chip_mapper_bc_0, 0))
        self.connect((self.wifi_dsss_ppdu_chip_mapper_bc_0, 0), (self.blocks_throttle2_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "simulation_txrx")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_rate(self):
        return self.rate

    def set_rate(self, rate):
        self.rate = rate
        self._rate_callback(self.rate)
        self.set_variable_qtgui_label_0("Long Preamble" if self.rate < 4 else "Short Preamble")
        self.wifi_dsss_chip_sync_c_0.set_preamble_type(self.rate<4)
        self.wifi_dsss_ppdu_prefixer_0.update_rate(self.rate)

    def get_variable_qtgui_label_0(self):
        return self.variable_qtgui_label_0

    def set_variable_qtgui_label_0(self, variable_qtgui_label_0):
        self.variable_qtgui_label_0 = variable_qtgui_label_0
        Qt.QMetaObject.invokeMethod(self._variable_qtgui_label_0_label, "setText", Qt.Q_ARG("QString", str(self._variable_qtgui_label_0_formatter(self.variable_qtgui_label_0))))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.blocks_throttle2_0.set_sample_rate(self.samp_rate)




def main(top_block_cls=simulation_txrx, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()
    tb.flowgraph_started.set()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
