import time
import numpy as np
import pybullet as p

class Object3D:
    """
    Representa um objeto 3D na simulação, incluindo a carga do modelo URDF,
    manipulação de posição, orientação, aplicação de forças e torques.
    """
    
    def __init__(self, urdf_file, start_position=[0, 0, 1], start_orientation=[0, 0, 0], mass=None):
        """
        Inicializa o objeto 3D com a localização do arquivo URDF e a posição inicial.
        
        :param urdf_file: caminho para o arquivo URDF.
        :param start_position: posição inicial como lista [x, y, z].
        :param start_orientation: orientação inicial como lista [roll, pitch, yaw].
        """
        self.mass = mass
        self.body_id = self.load_into_pybullet(urdf_file, start_position, start_orientation)

    def load_into_pybullet(self, urdf_file, start_position, start_orientation):
        """
        Carrega o objeto no PyBullet a partir do arquivo URDF.
        
        :return: ID do corpo no PyBullet.
        """
        body_id = p.loadURDF(urdf_file, start_position, p.getQuaternionFromEuler(start_orientation), globalScaling=1.0)
        if self.mass:
            p.changeDynamics(body_id, -1, mass=self.mass)
        return body_id

    def translate(self, dx, dy, dz):
        """
        Move o objeto por um vetor de deslocamento.
        
        :param dx: deslocamento em x.
        :param dy: deslocamento em y.
        :param dz: deslocamento em z.
        """
        current_pos, _ = p.getBasePositionAndOrientation(self.body_id)
        new_pos = np.array(current_pos) + np.array([dx, dy, dz])
        _, current_orn = p.getBasePositionAndOrientation(self.body_id)
        p.resetBasePositionAndOrientation(self.body_id, new_pos.tolist(), current_orn)

    def rotate(self, rx, ry, rz):
        """
        Rotaciona o objeto por ângulos especificados em radianos.
        
        :param rx: rotação em torno do eixo x.
        :param ry: rotação em torno do eixo y.
        :param rz: rotação em torno do eixo z.
        """
        current_pos, current_orn = p.getBasePositionAndOrientation(self.body_id)
        delta_orn = p.getQuaternionFromEuler([np.radians(rx), np.radians(ry), np.radians(rz)])
        new_orn = p.multiplyTransforms([0, 0, 0], current_orn, [0, 0, 0], delta_orn)[1]
        p.resetBasePositionAndOrientation(self.body_id, current_pos, new_orn)

    def set_position(self, pos):
        """
        Define a posição do objeto.
        
        :param pos: nova posição como lista [x, y, z].
        """
        _, current_orn = p.getBasePositionAndOrientation(self.body_id)
        p.resetBasePositionAndOrientation(self.body_id, pos, current_orn)

    def set_orientation(self, orn):
        """
        Define a orientação do objeto.
        
        :param orn: nova orientação como lista [roll, pitch, yaw].
        """
        current_pos, _ = p.getBasePositionAndOrientation(self.body_id)
        new_orn = p.getQuaternionFromEuler(orn)
        p.resetBasePositionAndOrientation(self.body_id, current_pos, new_orn)
    
    def apply_air_resistance(self):
        """
        Aplica uma força que simula a resistência do ar sobre o objeto.
        """
        if self.body_id is not None:
            velocity, _ = p.getBaseVelocity(self.body_id)
            drag_coefficient = -0.05  # Coeficiente de arrasto; ajuste conforme necessário para seu ambiente
            drag_force = np.array(velocity) * drag_coefficient
            p.applyExternalForce(self.body_id, -1, drag_force, [0, 0, 0], p.LINK_FRAME)
    
    def apply_force(self, force, position_rel_to_center=[0, 0, 0]):
        """
        Aplica uma força ao objeto. A força é um vetor [fx, fy, fz], e
        position_rel_to_center especifica o ponto de aplicação da força
        relativo ao centro de massa do objeto.
        """
        p.applyExternalForce(objectUniqueId=self.body_id,
                             linkIndex=-1,
                             forceObj=force,
                             posObj=position_rel_to_center,
                             flags=p.WORLD_FRAME)

    def apply_torque(self, torque):
        """
        Aplica um torque ao objeto. O torque é um vetor [tx, ty, tz].
        """
        p.applyExternalTorque(objectUniqueId=self.body_id,
                              linkIndex=-1,
                              torqueObj=torque,
                              flags=p.WORLD_FRAME)

    def set_velocity(self, linear_velocity=[0, 0, 0], angular_velocity=[0, 0, 0]):
        """
        Define a velocidade linear e angular do objeto.
        """
        p.resetBaseVelocity(self.body_id, linearVelocity=linear_velocity, angularVelocity=angular_velocity)


class Scene3D:
    """
    Representa a cena da simulação, incluindo objetos e métodos para adicionar ou remover objetos.
    """
    
    def __init__(self, gravity=-10):
        self.objects = []
        self.gravity = gravity
        p.connect(p.GUI)
        p.setGravity(0, 0, self.gravity)

    def add_object(self, object_3d, name=None):
        """
        Adiciona um objeto à cena.
        
        :param object_3d: instância de Object3D.
        :param name: nome opcional para o objeto.
        """
        self.objects.append((object_3d, name))

    def remove_object(self, name):
        """
        Remove um objeto da cena pelo nome.
        
        :param name: nome do objeto a ser removido.
        """
        self.objects = [obj for obj in self.objects if obj[1] != name]

    def update_objects_state(self):
        """
        Método de placeholder para atualizar o estado dos objetos na cena.
        Pode ser expandido para implementar comportamentos específicos.
        """
        p.stepSimulation()
        
        for obj, name in self.objects:
            obj.apply_air_resistance()
            # if name == "Ground":
            #     obj.rotate(0.03, 0, 0)
                
        time.sleep(1./240.)


def main():
    scene = Scene3D(-10)
    ground = Object3D('balanca_3d/objs/base.urdf', [0, 0, 0], [0, 0, 0])
    model = Object3D('balanca_3d/objs/bola.urdf', [0, 0, 5], [0, 0, 0])
    scene.add_object(ground, "Ground")
    scene.add_object(model, "Model")

    for _ in range(10000):
        scene.update_objects_state()

    p.disconnect()

if __name__ == "__main__":
    main()
