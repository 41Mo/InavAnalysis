# -*- coding: utf-8 -*-

###########################################################################
## Python code generated with wxFormBuilder (version Dec  5 2021)
## http://www.wxformbuilder.org/
##
## PLEASE DO *NOT* EDIT THIS FILE!
###########################################################################

import wx
import wx.xrc

###########################################################################
## Class MainFrame
###########################################################################

class MainFrame ( wx.Frame ):

	def __init__( self, parent ):
		wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = wx.EmptyString, pos = wx.DefaultPosition, size = wx.Size( 893,378 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )

		self.SetSizeHints( wx.DefaultSize, wx.DefaultSize )

		self.m_menubar1 = wx.MenuBar( 0 )
		self.m_menubar1.SetForegroundColour( wx.SystemSettings.GetColour( wx.SYS_COLOUR_WINDOWTEXT ) )

		self.m_menu1 = wx.Menu()
		self.m_menubar1.Append( self.m_menu1, u"MyMenu" )

		self.SetMenuBar( self.m_menubar1 )

		gSizer1 = wx.GridSizer( 2, 4, 0, 0 )

		gSizer2 = wx.GridSizer( 3, 2, 0, 0 )

		self.m_staticText7 = wx.StaticText( self, wx.ID_ANY, u"Координаты", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText7.Wrap( -1 )

		gSizer2.Add( self.m_staticText7, 0, wx.ALL, 5 )

		self.m_staticText8 = wx.StaticText( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText8.Wrap( -1 )

		gSizer2.Add( self.m_staticText8, 0, wx.ALL, 5 )

		self.m_staticText9 = wx.StaticText( self, wx.ID_ANY, u"долгота", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText9.Wrap( -1 )

		gSizer2.Add( self.m_staticText9, 0, wx.ALL, 5 )

		self.m_enter_lon = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		gSizer2.Add( self.m_enter_lon, 0, wx.ALL, 5 )

		self.m_staticText10 = wx.StaticText( self, wx.ID_ANY, u"широта", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText10.Wrap( -1 )

		gSizer2.Add( self.m_staticText10, 0, wx.ALL, 5 )

		self.m_enter_lat = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		gSizer2.Add( self.m_enter_lat, 0, wx.ALL, 5 )


		gSizer1.Add( gSizer2, 1, wx.EXPAND, 5 )

		gSizer3 = wx.GridSizer( 3, 2, 0, 0 )

		self.m_staticText11 = wx.StaticText( self, wx.ID_ANY, u"Параметры алгоритма", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText11.Wrap( -1 )

		gSizer3.Add( self.m_staticText11, 0, wx.ALL, 5 )

		self.m_staticText12 = wx.StaticText( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText12.Wrap( -1 )

		gSizer3.Add( self.m_staticText12, 0, wx.ALL, 5 )

		self.m_staticText13 = wx.StaticText( self, wx.ID_ANY, u"частота", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText13.Wrap( -1 )

		gSizer3.Add( self.m_staticText13, 0, wx.ALL, 5 )

		self.m_enter_freq = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		gSizer3.Add( self.m_enter_freq, 0, wx.ALL, 5 )

		self.m_staticText14 = wx.StaticText( self, wx.ID_ANY, u"кол-во точек", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText14.Wrap( 1 )

		gSizer3.Add( self.m_staticText14, 0, wx.ALL, 5 )

		self.m_enter_points_num = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		gSizer3.Add( self.m_enter_points_num, 0, wx.ALL, 5 )


		gSizer1.Add( gSizer3, 1, wx.EXPAND, 5 )

		gSizer4 = wx.GridSizer( 3, 2, 0, 0 )

		self.m_staticText15 = wx.StaticText( self, wx.ID_ANY, u"Дрейф", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText15.Wrap( -1 )

		gSizer4.Add( self.m_staticText15, 0, wx.ALL, 5 )

		self.m_staticText16 = wx.StaticText( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText16.Wrap( -1 )

		gSizer4.Add( self.m_staticText16, 0, wx.ALL, 5 )

		self.m_staticText17 = wx.StaticText( self, wx.ID_ANY, u"гироскопа", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText17.Wrap( -1 )

		gSizer4.Add( self.m_staticText17, 0, wx.ALL, 5 )

		self.m_enter_gyr_drift = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		gSizer4.Add( self.m_enter_gyr_drift, 0, wx.ALL, 5 )

		self.m_staticText18 = wx.StaticText( self, wx.ID_ANY, u"акс-ра", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText18.Wrap( -1 )

		gSizer4.Add( self.m_staticText18, 0, wx.ALL, 5 )

		self.m_enter_acc_drift = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		gSizer4.Add( self.m_enter_acc_drift, 0, wx.ALL, 5 )


		gSizer1.Add( gSizer4, 1, wx.EXPAND, 5 )

		gSizer5 = wx.GridSizer( 3, 2, 0, 0 )

		self.m_staticText19 = wx.StaticText( self, wx.ID_ANY, u"Смещение нуля", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText19.Wrap( -1 )

		gSizer5.Add( self.m_staticText19, 0, wx.ALL, 5 )

		self.m_staticText20 = wx.StaticText( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText20.Wrap( -1 )

		gSizer5.Add( self.m_staticText20, 0, wx.ALL, 5 )

		self.m_staticText21 = wx.StaticText( self, wx.ID_ANY, u"гироскопа", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText21.Wrap( -1 )

		gSizer5.Add( self.m_staticText21, 0, wx.ALL, 5 )

		self.m_enter_gyr_offset = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		gSizer5.Add( self.m_enter_gyr_offset, 0, wx.ALL, 5 )

		self.m_staticText22 = wx.StaticText( self, wx.ID_ANY, u"акс-ра", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText22.Wrap( -1 )

		gSizer5.Add( self.m_staticText22, 0, wx.ALL, 5 )

		self.m_enter_acc_offset = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		gSizer5.Add( self.m_enter_acc_offset, 0, wx.ALL, 5 )


		gSizer1.Add( gSizer5, 1, wx.EXPAND, 5 )

		self.m_staticText24 = wx.StaticText( self, wx.ID_ANY, u"Вид анализа", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText24.Wrap( -1 )

		gSizer1.Add( self.m_staticText24, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL, 5 )

		m_analysis_typeChoices = [ u"смещение нуля", u"дрейф" ]
		self.m_analysis_type = wx.Choice( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, m_analysis_typeChoices, 0 )
		self.m_analysis_type.SetSelection( 0 )
		gSizer1.Add( self.m_analysis_type, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 5 )

		self.m_button1 = wx.Button( self, wx.ID_ANY, u"Запуск", wx.DefaultPosition, wx.DefaultSize, 0 )
		gSizer1.Add( self.m_button1, 1, wx.ALIGN_RIGHT|wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )


		self.SetSizer( gSizer1 )
		self.Layout()

		self.Centre( wx.BOTH )

		# Connect Events
		self.m_button1.Bind( wx.EVT_BUTTON, self.run_analysis )

	def __del__( self ):
		pass


	# Virtual event handlers, overide them in your derived class
	def run_analysis( self, event ):
		event.Skip()


