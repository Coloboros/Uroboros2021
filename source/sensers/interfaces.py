from .out_data import OutData


class ISens:
    """
    Интерфейс для каждого датчика
    """
    def proc(self) -> OutData:
        """
        Обработка входных данных, датчиком
        """
        raise Exception("NotImplementedException")


__all__ = ['ISens']
