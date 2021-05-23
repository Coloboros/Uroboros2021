from source.sensers.geometry import Geometry

class Figure:
    def __init__(self, obj_name:str='unknown',
            geometry:Geometry=None, door:str='unknown'):
        self.obj_name = obj_name
        self.geometry = geometry if geometry != None else Geometry()
        self.door = door

    def to_object(self):
        res = {
            'object': self.obj_name,
            'geometry': self.geometry.to_object(),
            'door': self.door
        }
        return res
