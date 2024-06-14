script1 = [
    {
        'talker': 'robot',
        'script': 'Hello!'
    },
    {
        'talker': 'robot',
        'script': 'How are you?'
    }
]

scripts = [script1]

def get_script(number:int) -> list:
    if type(number) != int:
        return
    if 0 < number < len(scripts) + 1:
        return scripts[number - 1]
    else:
        return
    