from argparse import ArgumentParser
from subprocess import Popen
from time import sleep
import visdom

if __name__ == '__main__':
    parse = ArgumentParser()
    parse.add_argument('log_path', type=str)
    parse.add_argument('--host', type=str, default='localhost')
    parse.add_argument('--port', type=int, default=8097)
    args = parse.parse_args()

    vis_args = dict(
        server=args.host,
        port=args.port,
        raise_exceptions=True
    )
    try:
        vis = visdom.Visdom(**vis_args)
        vis.replay_log(args.log_path)
        while True:
            sleep(10)
    except ConnectionError:
        #try:
        cmd = f'visdom -port {args.port} -hostname {args.host} -eager_data_loading'
        print('Start a Visdom server first, i.e.')
        print(cmd)
            #print(cmd)
        #     p = Popen(f'visdom -port {args.port} --hostname {args.host} -eager_data_loading')
        #     vis = visdom.Visdom(**vis_args)
        #     vis.replay_log(args.log_path)
        #     while True:
        #         sleep(10)
        # except:
        #     p.terminate()
        #     p.kill()
        #     p.wait()