# Взаимодействие F207ZG и L4R5ZI через CAN шину

### Описание

**В данном репозитории хранится проект для L4R5ZI. [Проект для F207ZG](https://github.com/ducaton/NUCLEO-F207ZG_LED_through_CAN)**

Для проверки состояния подключения между платами пересылается пакет ID_CONNECT с частотой 1 Гц. Он отправляется в основном цикле и имеет погрешность в задержке. Если в течение 3 секунд (насчитывает таймер №8) плата не получила этот пакет, считается что произошёл разрыв на линии. В таком случае с частотой 2 Гц (насчитывает таймер №1) начинают мигать красный, синий и зелёный светодиоды. Также на Virtual Com Port посылаются уведомления о разрыве и о восстановлении соединения. Чтение данных возможно командой:

`cat /dev/ttyACM1`

Нажатие кнопки на плате формирует пакет ID_DATA, указывающий какие светодиоды переключить, и посылает его по CAN шине. Вторая плата должна принять пакет и переключить указанные светодиоды. Получение пакета реализовано через прерывание. Для устранения "дребезга" кнопки происходит её отключение после нажатия на 250 мс (насчитывает таймер №1).

Для управления какие светодиоды и на какой плате включать, реализована отправка пакетов с ПК по Virtual Com Port. Между платами этот пакет обладает именем ID_SET_LED. Для получения и обработки этого пакета также используется прерывание. Данные посылаются командой:

`echo -ne "\x1\x1\x1\x1\x1" > /dev/ttyACM1`

Каждый бит может быть либо 1, либо 0. Первые три бита отвечают за то, какие светодиоды теперь надо переключать. Остальные функции:

\x LD3 \x LD2 \x LD1 \x Отправить данные из USB на другую плату по CAN \x Записать данные из USB локально

При отправке данных ID_SET_LED на другую плату, у неё изменятся настройки, указывающие какие светодиоды переключать на первой. При записи локально, меняются настройки первой платы, указывающие на то какие светодиоды переключать на другой. 

В основном редактировались следующие файлы:
```
./Core/Src/main.c
./USB_DEVICE/App/usbd_cdc_if.c
./USB_DEVICE/App/usbd_cdc_if.h
```

Остальными файлами занималась STM32CubeIDE при изменении файла `./Loopback CAN 2.ioc `

### Инструкиця

1. Импортировать и собрать проект в STM32CubeIDE,
2. Подключить платы между собой через CAN шину и к компьютеру через USB,
3. Запустить проект.
