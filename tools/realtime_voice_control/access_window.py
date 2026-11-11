"""Small Ubuntu QR window; it does not depend on Firefox starting successfully."""
import urllib.request
import gi
gi.require_version('Gtk','3.0')
from gi.repository import Gtk,GdkPixbuf
from control_launcher import detect_ipv4


def main():
    address=detect_ipv4()
    try:
        with urllib.request.urlopen('http://127.0.0.1:8081/access.svg',timeout=5) as response:
            data=response.read()
    except OSError as error:
        dialog=Gtk.MessageDialog(message_type=Gtk.MessageType.ERROR,buttons=Gtk.ButtonsType.CLOSE,
                                 text='二维码服务尚未就绪')
        dialog.format_secondary_text('请重新启动控制台，或直接访问 https://'+address+':8080/\n'+str(error))
        dialog.run();dialog.destroy();return
    window=Gtk.Window(title='平衡车控制台 · 手机扫码')
    window.set_default_size(430,480)
    window.set_position(Gtk.WindowPosition.CENTER)
    window.set_keep_above(True)
    box=Gtk.Box(orientation=Gtk.Orientation.VERTICAL,spacing=12)
    box.set_border_width(24)
    box.pack_start(Gtk.Label(label='手机与电脑连接同一热点，扫码打开控制台'),False,False,0)
    loader=GdkPixbuf.PixbufLoader.new_with_type('svg');loader.set_size(280,280)
    loader.write(data);loader.close()
    box.pack_start(Gtk.Image.new_from_pixbuf(loader.get_pixbuf()),True,True,0)
    box.pack_start(Gtk.Label(label='https://'+address+':8080/'),False,False,0)
    box.pack_start(Gtk.Label(label='已信任过证书的手机无需重新安装\n连接后选择角色，人工复位后重新发令'),False,False,0)
    close=Gtk.Button(label='关闭窗口（控制服务继续运行）')
    close.connect('clicked',lambda button:window.close())
    box.pack_start(close,False,False,0)
    window.add(box);window.connect('destroy',Gtk.main_quit)
    window.show_all();window.present();Gtk.main()


if __name__=='__main__':main()
