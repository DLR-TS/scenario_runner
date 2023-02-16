import os.path

import cherrypy

config = {
    'global': {
        'server.socket_host': '0.0.0.0',
        'server.socket_port': 1999
    }
}


def upload(ufile, file_type):
    upload_path = os.path.dirname(__file__)
    if file_type == "scenario":
        upload_filename = 'setlevel_scenario.xosc'
    elif file_type == "map":
        upload_filename = 'setlevel_map.xodr'
    upload_file = os.path.normpath(
        os.path.join(upload_path, upload_filename)
    )
    size = 0
    with open(upload_file, 'wb') as out:
        while True:
            data = ufile.file.read(8192)
            if not data:
                break
            out.write(data)
            size += len(data)
    out = '''
    File received.
    Filename: {}
    Length: {}
    Mime-type: {}
    '''.format(ufile.filename, size, ufile.content_type, data)
    #TODO better transfer check
    if size == 0:
        os.remove(upload_filename)
    return out


def check_files_exist():
    return os.path.exists('setlevel_scenario.xosc') and os.path.exists('setlevel_map.xodr')


class App:

    @cherrypy.expose
    def scenario_upload(self, ufile):
        out = upload(ufile, "scenario")
        if check_files_exist():
            cherrypy.engine.exit()
        return out

    @cherrypy.expose
    def map_upload(self, ufile):
        out = upload(ufile, "map")
        if check_files_exist():
            cherrypy.engine.exit()
        return out

    @cherrypy.expose
    def exit(self):
        cherrypy.engine.exit()


if __name__ == '__main__':
    cherrypy.quickstart(App(), '/', config)
